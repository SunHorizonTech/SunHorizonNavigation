from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "sunhorizon_navigation_diagnostics"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sun Horizon Engineering",
    maintainer_email="46861254+dongchany@users.noreply.github.com",
    description="Read-only readiness probes for Sun Horizon navigation sensors.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "airy_readiness_probe = "
            "sunhorizon_navigation_diagnostics.airy_readiness_probe:main",
            "body_imu_readiness_probe = "
            "sunhorizon_navigation_diagnostics.body_imu_readiness_probe:main",
            "rgbd_readiness_probe = "
            "sunhorizon_navigation_diagnostics.rgbd_readiness_probe:main",
            "rgbd_depth_calibration_probe = "
            "sunhorizon_navigation_diagnostics.rgbd_depth_calibration_probe:main",
            "rgbd_alignment_capture = "
            "sunhorizon_navigation_diagnostics.rgbd_alignment_capture:main",
        ],
    },
)
