# Configuration file for the Sphinx documentation builder.

project = 'Community Robot Arm'
copyright = '2026, Community'
author = 'Community'

extensions = [
    'sphinx.ext.mathjax',
    'sphinx.ext.githubpages',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'sphinx_rtd_theme'
html_static_path = []
