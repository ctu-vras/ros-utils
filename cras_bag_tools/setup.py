from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['cras_bag_tools'],
    scripts=['scripts/bag_fix_msg_defs'],
    package_dir={'': 'src'}
)

setup(**d)
