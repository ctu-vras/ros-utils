# Configuration file for the Sphinx documentation builder.

import os
os.environ['CRAS_DOCS_COMMON_SPHINX_PACKAGE_PATH'] = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# By default, the ctu-vras theme will be used. To use your custom theme, set
# CRAS_DOCS_COMMON_SPHINX_THEME_PATH and CRAS_DOCS_COMMON_SPHINX_HTML_THEME

# include the master configuration
from cras_docs_common.sphinx_docs_conf import *