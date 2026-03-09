# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Top-level package for InOrbit Omron Connector."""

from importlib import metadata

__author__ = """InOrbit Inc."""
__email__ = "gerben@mappalink.com"
# Read the installed package version from metadata
try:
    __version__ = metadata.version("inorbit-omron-connector")
except metadata.PackageNotFoundError:
    __version__ = "unknown"
