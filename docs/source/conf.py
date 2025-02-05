#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################

import os
import subprocess
import sys


def get_git_revision_short_hash() -> str:
    """Get the short hash of the Git repository in the parent folder."""
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd="../").decode('ascii').strip()


# Path
sys.path.insert(0, os.path.abspath('../../pyolav/src'))

# Sphinx configuration
project = "OLAV"
copyright = "2023, Aarhus Robotics"
author = "Dario Sirangelo"
release = "1.0.0"
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.graphviz', 'sphinx.ext.todo']
templates_path = ["_templates"]

# Language configuration
primary_domain = 'cpp'
highlight_language = 'cpp'

# HTML build configuration
html_theme = "furo"
html_static_path = ['_static']
html_css_files = [
    'css/custom.css',
]
html_title = "OLAV"
html_logo = "logo.png"
html_context = {"default_mode": "dark"}
html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#00315c",
        "color-brand-content": "#00315c",
    },
}

# LaTeX build configuration
latex_elements = {
    'extraclassoptions': 'openany,oneside',
    "releasename": "Commit hash",
}
latex_authors = r"Dario Sirangelo"
latex_documents = [("index", "olav.tex", project, latex_authors, "manual")]
pdf_documents = [("index", "olav", "title", latex_authors)]

# Autodoc extension configuration
autodoc_default_options = {
    'special-members': '__init__',
    'members': True,
    'private-members': True,
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

graphviz_output_format = 'svg'

# Todo extension configuration
todo_include_todos = True
