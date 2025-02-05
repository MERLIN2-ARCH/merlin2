from setuptools import setup, find_packages

package_name = "merlin2_executor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Miguel Ángel González Santamarta",
    maintainer_email="mgonzs13@estudiantes.unileon.es",
    description="MERLIN2 executor package",
    license="GPL-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "executor_node = merlin2_executor.merlin2_executor_node:main",
        ],
    },
)
