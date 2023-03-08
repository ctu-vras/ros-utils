# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""A default conf.py file for rosdoc_lite/sphinx documentation builder.

It provides some useful shortcuts and automatically links to ROS message definitions on docs.ros.org.

You can try e.g. :roswiki:`cras_py_common`, :rosdep:`rosdep2_api.html`, :tf2_msgs:`TFMessage`, :class:`genpy.Message` or
:class:`tf2_msgs.msg.TFMessage`.

To use this file in you project, create file `doc/conf.py` with the following content::

  import os
  os.environ['CRAS_DOCS_COMMON_SPHINX_PACKAGE_PATH'] = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
  from cras_docs_common.sphinx_docs_conf import *

  # Here you can customize all the variables, but leaving the rest empty is usually enough.

"""

import os
import sys
import time
import catkin_pkg.package

from os.path import abspath, dirname, join

from docutils import nodes
from sphinx.ext import intersphinx

# Set this variable to pass the path to the package root (i.e. the folder containing package.xml).
catkin_dir = os.getenv('CRAS_DOCS_COMMON_SPHINX_PACKAGE_PATH', dirname(dirname(abspath(__file__))))
catkin_package = catkin_pkg.package.parse_package(join(catkin_dir, catkin_pkg.package.PACKAGE_MANIFEST_FILENAME))

project = catkin_package.name
copyright = time.strftime("%Y") + ', Czech Technical University in Prague'
author = ", ".join([a.name for a in catkin_package.authors])
version = catkin_package.version
release = catkin_package.version

import catkin_sphinx

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.extlinks',
    'catkin_sphinx.ShLexer',
    'catkin_sphinx.cmake',
]

try:
    import m2r
    extensions.append('m2r')
except ImportError:
    pass

master_doc = 'index'
autoclass_content = 'both'

this_dir = os.path.abspath(os.path.dirname(os.path.abspath(__file__)))
theme_path = os.path.join(this_dir, 'sphinx-themes')

html_theme_path = [theme_path, catkin_sphinx.get_theme_path()]

# Use ROS theme
html_theme = 'cras-theme'

ros_distro = os.getenv('ROS_DISTRO', 'latest')
ros_api_base = 'https://docs.ros.org/en/%s/api/'
ros_wiki_base = 'https://wiki.ros.org/'

# Dynamic project title
rst_epilog = """
.. |project| replace:: {}
.. |description| replace:: {}
.. role:: raw-html(raw)
   :format: html

.. |wiki| replace:: :raw-html:`<a href="{}{}?distro={}">ROS wiki entry</a>`
""".format(project, catkin_package.description, ros_wiki_base, project, ros_distro)

ros_api = ros_api_base % ros_distro
melodic_api = ros_api_base % "melodic"
independent_api = ros_api_base % "independent"

extlinks = {
    'ros': (ros_api + '%s', None),
    'roswiki': (ros_wiki_base + '%s?distro=' + ros_distro, ''),
    'rospy:class': (melodic_api + 'rospy/html/rospy.%s-class.html', "rospy."),
    'rospy:fn': (melodic_api + 'rospy/html/rospy-module.html#%s', "rospy."),
    'catkin_pkg': (independent_api + 'catkin_pkg/html/%s', None),
    'rosdep': (independent_api + 'rosdep/html/%s', None),
    'rospkg': (independent_api + 'rospkg/html/%s', None),
    'dynamic_reconfigure': (ros_api + 'dynamic_reconfigure/html/msg/%s.html', "dynamic_reconfigure/"),
}
for dep in catkin_package.exec_depends:
    if dep.name.endswith("_msgs"):
        extlinks[dep.name] = (ros_api + dep.name + '/html/msg/%s.html', dep.name + "/")
    elif dep.name.endswith("_srvs"):
        extlinks[dep.name] = (ros_api + dep.name + '/html/srv/%s.html', dep.name + "/")
    elif dep.name not in extlinks:
        extlinks[dep.name] = (ros_api + dep.name + '/html/%s.html', dep.name + "/")

intersphinx_mapping = {
    "numpy": ("https://numpy.org/doc/stable/", None),
    "genpy": (ros_api + "genpy/html", None),
}
if sys.version_info.major == 2:
    intersphinx_mapping['python'] = ('https://docs.python.org/2.7/', None)
else:
    python_version = '{}.{}'.format(sys.version_info.major, sys.version_info.minor)
    intersphinx_mapping['python'] = ('https://docs.python.org/{}/'.format(python_version), None)


def ros_msg_reference(app, env, node, contnode):
    text_node = next(iter(contnode.traverse(lambda n: n.tagname == '#text')))
    parts = text_node.astext().split('.')
    if len(parts) != 3:
        return intersphinx.missing_reference(app, env, node, contnode)
    pkg, obj_type, obj = parts
    if obj_type not in ("msg", "srv"):
        return intersphinx.missing_reference(app, env, node, contnode)
    target = ros_api + '{}/html/{}/{}.html'.format(pkg, obj_type, obj)
    ref_node = nodes.reference()
    ref_node['refuri'] = target
    title = '{}/{}'.format(pkg, obj)
    text_node = nodes.literal(title, title)
    text_node['classes'] = ['xref', 'ros', 'ros-' + obj_type]
    ref_node += text_node
    return ref_node

# Backport of fix for issue https://github.com/sphinx-doc/sphinx/issues/2549. Without it, :ivar: fields wrongly resolve cross-references.
from sphinx.domains.python import PyObject
PyObject.doc_field_types[map(lambda f: f.name == 'variable', PyObject.doc_field_types).index(True)].rolename = None

def setup(app):
    app.connect("missing-reference", ros_msg_reference)
