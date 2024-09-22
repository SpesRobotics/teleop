from setuptools import setup, find_packages

from pathlib import Path
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text()

setup(
    name='teleop',
    version='0.0.1',
    packages=find_packages(),
    long_description=long_description,
    long_description_content_type='text/markdown',
    install_requires=[
        'Flask',
        'numpy',
        'transforms3d',
        'werkzeug',
        'pytest',
        'requests',
    ],
    package_data={
        'teleop': ['cert.pem', 'key.pem', 'index.html'],
    },
)
