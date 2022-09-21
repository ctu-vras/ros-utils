## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague


from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cras', 'cras.impl'],
    package_dir={'': 'src'},
)

setup(**setup_args)
