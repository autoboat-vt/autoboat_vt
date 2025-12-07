"""
Module for syntax highlighter classes in the ground station application.

Contains:
- BaseHighlighter: Base class for syntax highlighters.
- ConsoleHighlighter: Syntax highlighter for console output text.
- JsonHighlighter: Syntax highlighter for JSON text.
"""

__all__ = ["BaseHighlighter", "ConsoleHighlighter", "JsonHighlighter"]

from .base_highlighter import BaseHighlighter
from .console import ConsoleHighlighter
from .json import JsonHighlighter
