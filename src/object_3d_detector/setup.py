from setuptools import setup

package_name = "object_3d_detector"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franco",
    maintainer_email="franco@example.com",
    description="YOLO + Kinect depth detector publishing geometry_msgs/Point",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_depth_to_point = object_3d_detector.yolo_depth_to_point:main",
        ],
    },
)
