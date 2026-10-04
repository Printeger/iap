"""Private entrypoint for the current EGO simulation graph."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from full_stack_runtime import generate_launch_description, planner_parameters
