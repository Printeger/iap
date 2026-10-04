"""Flight entrypoint reserved until EGO rebuild execution stages are complete."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction


def _setup(context):
    raise RuntimeError(
        "EGO rebuild stage 1 has no integrity trajectory check or continuous "
        "execution handoff yet; iap_flight remains unavailable until stages 4/5. "
        "Deployment calibration and controller handshake must be revalidated then."
    )


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
