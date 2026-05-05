from setuptools import setup

package_name = "iap_phase1_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iap",
    maintainer_email="dev@iap.local",
    description="IAP/EGO closed-loop logging and trajectory integrity evaluation helpers.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "phase1_closed_loop_logger = iap_phase1_tools.phase1_closed_loop_logger:main",
            "phase2_planner_integrity_evaluator = iap_phase1_tools.phase2_planner_integrity_evaluator:main",
        ],
    },
)
