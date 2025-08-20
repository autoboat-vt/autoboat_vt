"""
Module for syntax highlighter classes in the ground station application.

Contains:
- BaseHighlighter: Base class for syntax highlighters.
- JsonHighlighter: Syntax highlighter for JSON text.
- ConsoleHighlighter: Syntax highlighter for console output text.
"""

__all__ = ["BaseHighlighter", "ConsoleHighlighter", "JsonHighlighter"]

from .base_highlighter import BaseHighlighter
from .json import JsonHighlighter
from .console import ConsoleHighlighter
