import re
from pathlib import Path
from setuptools import setup


this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

# replace relative links with absolute links
long_description = re.sub(
    r"\]\((?!http)([^)]+)\)",
    r"](https://github.com/SpesRobotics/teleop/raw/main/\1)",
    long_description,
)


setup(
    name="teleop",
    version="0.0.3",
    packages=["teleop", "teleop.basic", "teleop.ros2"],
    long_description=long_description,
    long_description_content_type="text/markdown",
    description="Turns your phone into a robot arm teleoperation device by leveraging the WebXR API",
    install_requires=[
        "Flask",
        "numpy",
        "transforms3d",
        "werkzeug",
        "pytest",
        "requests",
    ],
    package_data={
        "teleop": ["cert.pem", "key.pem", "index.html"],
    },
    license="Apache 2.0",
    author="Spes Robotics",
    author_email="contact@spes.ai",
    project_urls={
        "Documentation": "https://github.com/SpesRobotics/teleop",
        "Source": "https://github.com/SpesRobotics/teleop",
        "Tracker": "https://github.com/SpesRobotics/teleop/issues",
    },
)
