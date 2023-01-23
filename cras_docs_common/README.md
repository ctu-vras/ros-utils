# cras\_docs\_common

Common configuration for CRAS packages documentation.

## Sphinx

To document your Python package with Sphinx, add this to your `package.xml`:

```XML
<doc_depend>cras_docs_common</doc_depend>
<doc_depend>rosdoc_lite</doc_depend>
<export>
    <rosdoc config="rosdoc.yaml"/>
</export>
```

Create file `rosdoc.yaml` with the following content:

```YAML
- builder: sphinx
  sphinx_root_dir: doc
```

Create folder `doc`, in it create file `doc/conf.py` with the following content:

```python
# Configuration file for the Sphinx documentation builder.

import os
os.environ['CRAS_DOCS_COMMON_SPHINX_PACKAGE_PATH'] = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
from cras_docs_common.sphinx_docs_conf import *
```

Create `doc/index.rst` with this content:

```rst
.. documentation master file

|project|
===============================================================================

|description|

Python API
----------

.. toctree::
   :maxdepth: 2

   modules

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
```

Finally, create file `doc/modules.rst` containing the actual modules to be documented, i.e.

```rst
cras package
============

Submodules
----------

cras.ctypes\_utils module
-------------------------

.. automodule:: cras.ctypes_utils
   :members:
   :undoc-members:
   :show-inheritance:
```

The Sphinx config from this package provides some useful shortcuts and automatically links to ROS message definitions on docs.ros.org.

You can try e.g. ``:roswiki:`cras_py_common` ``, ``:rosdep:`rosdep2_api.html` ``, ``:tf2_msgs:`TFMessage` ``, ``:class:`genpy.Message` `` or
``:class:`tf2_msgs.msg.TFMessage` ``.

It also automatically loads the catkin_sphinx extensions [ShLexer](https://github.com/ros-infrastructure/catkin-sphinx/#using-an-improved-shell-prompt-highlighting) and [cmake](https://github.com/ros-infrastructure/catkin-sphinx/#using-the-cmake-sphinx-domain).