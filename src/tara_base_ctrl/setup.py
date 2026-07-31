from setuptools import find_packages, setup


package_name = "tara_base_ctrl"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "minimalmodbus>=2.1.1"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="ROS 2 velocity control for the Tara differential-drive base",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tara_base_node = tara_base_ctrl.tara_base_node:main",
        ],
    },
)
