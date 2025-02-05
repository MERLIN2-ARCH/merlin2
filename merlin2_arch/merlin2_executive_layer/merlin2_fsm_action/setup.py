from setuptools import setup, find_packages

package_name = "merlin2_fsm_action"

setup(
    name=package_name,
    version="2.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgons@unileon.es",
    description="FSM actions for MERLIN2",
    license="GPL-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
