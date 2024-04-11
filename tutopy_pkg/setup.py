from setuptools import find_packages, setup

package_name = "tutopy_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gonca",
    maintainer_email="gonca@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "py_node = tutopy_pkg.sample_node:main",
            "publisher_node = tutopy_pkg.publisher_node:main",
            "subscriber_node = tutopy_pkg.subscriber_node:main",
            "server_node = tutopy_pkg.server_node:main",
            "client_node = tutopy_pkg.client_node:main",
            "hw_status_publisher = tutopy_pkg.hw_status_publisher:main",
            "hw_status_subscriber = tutopy_pkg.hw_status_subscriber:main",
            "LED_client_node = tutopy_pkg.activity_04_client:main"
        ],
    },
)
