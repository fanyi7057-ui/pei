from setuptools import find_packages, setup

package_name = "hh"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/hh"]),
        ("share/hh", ["package.xml"]),
        ("share/hh/config", ["config/hh_test.yaml"]),
        ("share/hh/launch", [
            "launch/hh_visual_test.launch.py",
            "launch/hh_step_test.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@example.com",
    description="Independent ICAN task-3 test center",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "hh_menu = hh.menu:main",
        "hh_visual = hh.visual:main",
        "hh_state_machine = hh.state_machine:main",
    ]},
)
