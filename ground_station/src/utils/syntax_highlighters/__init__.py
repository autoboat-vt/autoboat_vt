"""
Package for syntax highlighter classes in the Groundstation application.

Contains:
- BaseHighlighter: Base class for syntax highlighters.
- ConsoleHighlighter: Syntax highlighter for console output text.
- JsonHighlighter: Syntax highlighter for JSON text.
"""

__all__ = ["ConsoleHighlighter", "JsonHighlighter"]

from .console import ConsoleHighlighter
from .json import JsonHighlighter
