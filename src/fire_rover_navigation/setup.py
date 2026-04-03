from setuptools import find_packages, setup

package_name = "fire_rover_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "ultralytics", "torch"],
    zip_safe=True,
    maintainer="zygar",
    maintainer_email="adrianzygar@wp.pl",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "goal_sender = fire_rover_navigation.goal_sender:main",
            "base_alignment_node = fire_rover_navigation.base_alignment_node:main",
        ],
    },
)
