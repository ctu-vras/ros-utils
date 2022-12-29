# Configuration file for the Sphinx documentation builder.

import os
import time
import catkin_pkg.package
catkin_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
catkin_package = catkin_pkg.package.parse_package(os.path.join(catkin_dir, catkin_pkg.package.PACKAGE_MANIFEST_FILENAME))

project = catkin_package.name
copyright = time.strftime("%Y") + ', Czech Technical University in Prague'
author = ", ".join([a.name for a in catkin_package.authors])
version = catkin_package.version
release = catkin_package.version

extensions = ['sphinx.ext.autodoc']

master_doc = 'index'
autoclass_content = 'both'

html_theme = 'default'
