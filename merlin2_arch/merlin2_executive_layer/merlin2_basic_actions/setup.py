from setuptools import setup, find_packages

package_name = "merlin2_basic_actions"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgons@unileon.es",
    description="Basic actions of MERLIN2",
    license="GPL-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "merlin2_navigation_action = merlin2_basic_actions.merlin2_navigation_action:main",
            "merlin2_navigation_fsm_action = merlin2_basic_actions.merlin2_navigation_fsm_action:main"
        ],
    },
)
