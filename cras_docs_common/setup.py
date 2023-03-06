## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague


from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['cras_docs_common'],
    package_dir={'': 'src'},
    # The following is needed to provide the theme files for the other packages in this stack during doc jobs on buildfarm (it ignores the install() directive in CMakeLists.txt)
    package_data={'cras_docs_common': ['sphinx-themes/*', 'sphinx-themes/*/*', 'sphinx-themes/*/*/*', 'sphinx-themes/*/*/*/*']},
)

setup(**setup_args)
