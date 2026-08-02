#!/usr/bin/env python3
"""Record an evidence bag and finalize its manifest in one process lifetime.

ROS launch sends SIGINT during the duration shutdown.  An ``OnProcessExit``
handler is not reliable at that point because launch is already tearing down
its action graph.  This wrapper owns the recorder child, waits for rosbag to
flush metadata, and only then invokes the manifest finalizer.
"""

import argparse
import signal
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--bag-output", required=True)
    parser.add_argument("topics", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    topics = list(args.topics)
    if topics[:1] == ["--"]:
        topics = topics[1:]
    if not topics:
        raise SystemExit("at least one bag topic is required")

    recorder_command = ["ros2", "bag", "record", "-o", args.bag_output, *topics]
    print("[planner_bag_recorder] command=" + " ".join(recorder_command), flush=True)
    recorder = subprocess.Popen(recorder_command)
    stopping = False

    def request_stop(signum, _frame):
        nonlocal stopping
        if not stopping:
            stopping = True
            if recorder.poll() is None:
                recorder.send_signal(signum)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    recorder_exit = recorder.wait()
    print(f"[planner_bag_recorder] exit_code={recorder_exit}", flush=True)

    finalizer = Path(__file__).with_name("finalize_planner_evidence_manifest.py")
    finalized = subprocess.run(
        [sys.executable, str(finalizer), "--manifest", args.manifest,
         "--wait-timeout-s", "15", "--recorder-exit-code", str(recorder_exit),
         "--recorder-command", " ".join(recorder_command)],
        check=False,
    )
    if finalized.returncode != 0:
        return finalized.returncode
    return recorder_exit


if __name__ == "__main__":
    sys.exit(main())
