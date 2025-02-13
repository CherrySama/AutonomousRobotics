from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# from setuptools import setup, find_packages

# setup(
#     name="minitask5",
#     version="0.0.0",
#     packages=find_packages(),
#     scripts=["main.py"],
#     install_requires=["rospy"],
# )
packages_required = generate_distutils_setup(
    packages=['minitask5/scripts/modules'],
    package_dir={'': 'scripts'}
)

setup(**packages_required)
