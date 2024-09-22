from setuptools import setup, find_packages

setup(
    name='teleop',
    version='0.1',
    packages=find_packages(),
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
