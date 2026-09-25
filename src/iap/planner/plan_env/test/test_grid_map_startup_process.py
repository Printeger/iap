import os
import re
import signal
import subprocess
import time
import unittest


class GridMapStartupProcessTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = os.environ["GRID_MAP_STARTUP_PROCESS_PROBE"]

    def run_scenario(self, domain_id, producer_delay_s, consumer_delay_s,
                     producer_mode=None, expected_published_deltas=1,
                     controlled_default_load=False,
                     expect_recovery_commit=True):
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
                    consumer_args = ["5.0"]
                    if controlled_default_load:
                        consumer_args.append("controlled-default-load")
                    consumer = start("consumer", *consumer_args)
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
                r"recovery request request_serial=[1-9][0-9]* "
                r"request_base_generation=[0-9]+ "
                r"observed_generation=[0-9]+ committed_generation=[0-9]+ "
                r"service_ready=1",
            )
            if expect_recovery_commit:
                self.assertRegex(
                    timeline,
                    r"recovery response request_serial=[1-9][0-9]* .*"
                    r"response_complete=1 response_generation=[0-9]+ "
                    r"commit_result=committed reject_reason=none",
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
        self.run_scenario(
            171, producer_delay_s=0.8, consumer_delay_s=0.0,
            expect_recovery_commit=False)

    def test_producer_before_delayed_consumer(self):
        timeline = self.run_scenario(
            172, producer_delay_s=0.0, consumer_delay_s=0.8)
        self.assertRegex(
            timeline,
            r"recovery_response serial=[0-9]+ generation=1 "
            r"producer_serial=[2-9][0-9]* active_serial=1 complete=1")
        self.assertIn("pending_source_y=0.75", timeline)
        self.assertIn("active_source_y=0", timeline)

    def test_recovery_after_first_request_timeout(self):
        timeline = self.run_scenario(
            173,
            producer_delay_s=0.0,
            consumer_delay_s=0.8,
            producer_mode="first-response-timeout",
            expected_published_deltas=2,
        )
        self.assertIn("recovery timeout request_serial=1", timeline)
        self.assertIn("recovery_response serial=1 generation=1", timeline)
        self.assertRegex(
            timeline,
            r"committed_generation=2 .*commit_result=committed "
            r"reject_reason=none pending=0")
        self.assertNotIn(
            "recovery response request_serial=1", timeline)

    def test_response_behind_observed_commits_then_catches_up(self):
        timeline = self.run_scenario(
            174,
            producer_delay_s=0.0,
            consumer_delay_s=0.8,
            producer_mode="response-behind-observed",
            expected_published_deltas=2,
        )
        self.assertRegex(
            timeline,
            r"response request_serial=1 .*observed_generation=2 "
            r"committed_generation=1 .*pending=1")
        self.assertRegex(
            timeline,
            r"response request_serial=2 .*observed_generation=2 "
            r"committed_generation=2 .*pending=0")
        self.assertIn("active_generation=2", timeline)
        self.assertIn("active_source_y=0.25", timeline)

    def test_late_old_response_cannot_suppress_current_timeout(self):
        timeline = self.run_scenario(
            177,
            producer_delay_s=0.0,
            consumer_delay_s=0.8,
            producer_mode="late-old-response",
        )
        self.assertIn("recovery timeout request_serial=1", timeline)
        self.assertRegex(
            timeline,
            r"recovery timeout request_serial=3 .*"
            r"response_received_steady=[1-9][0-9]* .*"
            r"reject_reason=uncorrelated_or_undispatched_response")
        self.assertRegex(
            timeline,
            r"recovery response request_serial=5 .*"
            r"commit_result=committed reject_reason=none")

    def test_recovery_response_is_not_starved_by_default_planner_load(self):
        timeline = self.run_scenario(
            175,
            producer_delay_s=0.0,
            consumer_delay_s=0.8,
            producer_mode="callback-starvation",
            controlled_default_load=True,
        )
        self.assertIn("default_callback_load_started", timeline)
        self.assertIn("default_callback_load_finished", timeline)
        self.assertIn("recovery_response serial=1 generation=1", timeline)
        self.assertIn("recovery response request_serial=1", timeline)
        self.assertIn("commit_result=committed reject_reason=none", timeline)
        self.assertNotIn("recovery timeout", timeline)
        callback_timing_match = re.search(
            r"recovery response request_serial=1 .*"
            r"request_sent_steady=([0-9]+) "
            r"response_received_steady=([0-9]+) "
            r"callback_started_steady=([0-9]+) "
            r"timeout_steady=([0-9]+)", timeline)
        self.assertIsNotNone(callback_timing_match)
        request_sent, response_received, callback_started, deadline = (
            int(value) for value in callback_timing_match.groups())
        self.assertLess(request_sent, response_received)
        self.assertLessEqual(response_received, callback_started)
        self.assertLess(callback_started, deadline)
        serial_match = re.search(
            r"producer_serial=([0-9]+) active_serial=([0-9]+)", timeline)
        response_sent_match = re.search(
            r"response_sent_steady=([0-9]+)", timeline)
        timeout_match = re.search(r"timeout_steady=([0-9]+)", timeline)
        load_finished_match = re.search(
            r"default_callback_load_finished steady=([0-9]+)", timeline)
        self.assertIsNotNone(serial_match)
        self.assertIsNotNone(response_sent_match)
        self.assertIsNotNone(timeout_match)
        self.assertIsNotNone(load_finished_match)
        server_incomplete = (
            int(serial_match.group(1)) > int(serial_match.group(2)))
        callback_starvation = (
            int(response_sent_match.group(1)) < int(timeout_match.group(1)) <
            int(load_finished_match.group(1)))
        causes = []
        if server_incomplete:
            causes.append("SERVER_INCOMPLETE")
        if callback_starvation:
            causes.append("CLIENT_CALLBACK_STARVATION")
        first_cause = (
            "BOTH" if len(causes) == 2 else
            (causes[0] if causes else "NONE"))
        self.assertEqual(first_cause, "BOTH")
        print(f"FIRST_CAUSE={first_cause}")

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
