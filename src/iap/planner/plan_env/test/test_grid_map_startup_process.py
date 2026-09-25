import os
import signal
import subprocess
import time
import unittest


class GridMapStartupProcessTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = os.environ["GRID_MAP_STARTUP_PROCESS_PROBE"]

    def run_scenario(self, domain_id, producer_delay_s, consumer_delay_s,
                     producer_mode=None, expected_published_deltas=1):
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(domain_id)
        processes = []

        def start(role, *args):
            process = subprocess.Popen(
                [self.probe, role, *args],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            processes.append(process)
            return process

        producer_output = ""
        try:
            started = time.monotonic()
            producer = None
            consumer = None
            while producer is None or consumer is None:
                elapsed = time.monotonic() - started
                if consumer is None and elapsed >= consumer_delay_s:
                    consumer = start("consumer", "7.0")
                if producer is None and elapsed >= producer_delay_s:
                    modes = [producer_mode] if producer_mode else []
                    producer = start("producer", *modes)
                time.sleep(0.01)

            consumer_output, _ = consumer.communicate(timeout=9.0)
            if producer.poll() is None:
                os.killpg(producer.pid, signal.SIGTERM)
            producer_output, _ = producer.communicate(timeout=2.0)
            timeline = producer_output + consumer_output
            self.assertEqual(
                consumer.returncode,
                0,
                f"consumer failed in ROS_DOMAIN_ID={domain_id}:\n{timeline}",
            )
            self.assertIn("current_endpoint_matched", timeline)
            self.assertIn("first_current_published", timeline)
            self.assertIn("first_delta base=0 generation=1", timeline)
            self.assertIn("recovery_service_ready", timeline)
            self.assertIn("recovery_request serial=", timeline)
            self.assertIn("recovery_response serial=", timeline)
            self.assertRegex(
                timeline,
                r"recovery service ready request_serial=[1-9][0-9]* "
                r"request_base=[0-9]+ observed=[0-9]+",
            )
            self.assertRegex(
                timeline,
                r"recovery committed request_serial=[1-9][0-9]* "
                r"response=[0-9]+ observed=[0-9]+ committed=[0-9]+ "
                r"pending=[01]",
            )
            self.assertIn("snapshot=present", timeline)
            self.assertRegex(timeline, r"generation=[1-9][0-9]*")
            self.assertRegex(timeline, r"active_generation=[1-9][0-9]*")
            self.assertIn(
                f"RESULT producer generation={expected_published_deltas} "
                f"published_deltas={expected_published_deltas}",
                timeline,
            )
            if consumer_delay_s >= 0.8:
                self.assertRegex(timeline, r"unchanged_attempts=[1-9][0-9]*")
            return timeline
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
            for process in reversed(processes):
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.0)

    def test_consumer_before_delayed_producer_and_service(self):
        self.run_scenario(171, producer_delay_s=0.8, consumer_delay_s=0.0)

    def test_producer_before_delayed_consumer(self):
        self.run_scenario(172, producer_delay_s=0.0, consumer_delay_s=0.8)

    def test_recovery_after_first_request_timeout(self):
        timeline = self.run_scenario(
            173,
            producer_delay_s=0.0,
            consumer_delay_s=0.8,
            producer_mode="first-response-timeout",
            expected_published_deltas=2,
        )
        self.assertIn("recovery timed out; retrying", timeline)
        self.assertIn("recovery_response serial=1 generation=1", timeline)
        self.assertIn("committed=2 pending=0", timeline)
        self.assertNotIn(
            "recovery committed request_serial=1", timeline)

    def test_response_behind_observed_commits_then_catches_up(self):
        timeline = self.run_scenario(
            174,
            producer_delay_s=0.0,
            consumer_delay_s=0.8,
            producer_mode="response-behind-observed",
            expected_published_deltas=2,
        )
        self.assertIn("response=1 observed=2 committed=1 pending=1", timeline)
        self.assertIn("response=2 observed=2 committed=2 pending=0", timeline)
        self.assertIn("active_generation=2", timeline)
        self.assertIn("active_source_y=0.25", timeline)

    def test_quick_producer_first_startup_ten_times(self):
        for repetition in range(10):
            with self.subTest(repetition=repetition):
                self.run_scenario(
                    180 + repetition,
                    producer_delay_s=0.0,
                    consumer_delay_s=0.8,
                )

if __name__ == "__main__":
    unittest.main()
