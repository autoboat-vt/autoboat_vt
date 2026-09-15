"""
Serve the built map-widget frontend from an in-process HTTP server.

Replaces the Vite dev server at runtime: the app reads the same files that
``vite build`` emits into ``app_data/frontend`` (which is also what the Vite
dev server used as its ``publicDir`` — all bundled assets land under
``git_keep/assets``). This keeps the packaged app fully self-contained with
no Node/Bun dependency.

Because the in-app servers all live on loopback ports owned by this process,
the asset server also serves these frontend files directly: the map page is
loaded from ``http://127.0.0.1:<asset_port>/frontend/index.html``.
"""

from __future__ import annotations

import http.server
import mimetypes
import socketserver
from pathlib import Path

from utils.console_logger import get_logger

__all__ = ["FRONTEND_DIR", "mounted_frontend_dir", "run"]

logger = get_logger(__name__)

FRONTEND_DIR = Path(__file__).resolve().parent / "dist"


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    """Serve a fixed directory without per-request console spam."""

    def log_message(self, format_string: str, *args: object) -> None:
        pass


def mounted_frontend_dir() -> Path | None:
    """
    Return the directory containing the built frontend when the generated
    files have been copied into the Vite ``publicDir``‐style layout expected
    by the asset server (``app_data/frontend``).

    Returns
    -------
    :class:`Path` | `None`
        The mounted directory, or `None` when the built frontend has never
        been generated (fresh source checkout without a build step).
    """

    from utils import constants

    mounted = Path(constants.DATA_DIR) / "frontend"
    return mounted if (mounted / "index.html").is_file() else None


def run(port: int, directory: Path) -> None:
    """
    Serve ``directory`` over HTTP on ``127.0.0.1:port`` forever.

    Parameters
    ----------
    port
        The TCP port to bind.
    directory
        The directory whose contents are served as the site root.
    """

    def handler(*args: tuple, **kwargs: dict) -> _QuietHandler:
        return _QuietHandler(*args, directory=directory.as_posix(), **kwargs)

    mimetypes.add_type("text/javascript", ".js")
    mimetypes.add_type("text/css", ".css")
    mimetypes.add_type("image/svg+xml", ".svg")

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("127.0.0.1", port), handler) as server:
        logger.info(f"Serving map frontend from {directory} on port {port}...")
        server.serve_forever()
