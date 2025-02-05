from setuptools import setup

package_name = "merlin2_mission"

setup(
    name=package_name,
    version="2.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="miguel",
    maintainer_email="mgons@unileon.es",
    description="MERLIN2 mission package",
    license="GPL-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
