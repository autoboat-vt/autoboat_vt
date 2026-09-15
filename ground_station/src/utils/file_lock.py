from __future__ import annotations

import sys
from typing import IO, Any

__all__ = ["lock_exclusive", "lock_shared", "unlock"]

if sys.platform == "win32":
    import msvcrt

    _LOCK_NBYTES = 1
    _LOCK_TIMEOUT_SECS = 10

    def _prepare(file: IO[Any]) -> None:
        """
        Prepare a file for locking by flushing and seeking to the beginning.
        This is necessary on Windows to ensure that the lock is applied to the
        correct portion of the file.

        Parameters
        ----------
        file
            The file object to prepare for locking.
        """

        file.flush()
        file.seek(0)

    def lock_shared(file: IO[Any]) -> None:
        """
        Acquire a shared (read) lock via :func:`msvcrt.locking`.

        Parameters
        ----------
        file
            The file object to lock.
        """

        _prepare(file)
        msvcrt.locking(file.fileno(), msvcrt.LK_LOCK, _LOCK_NBYTES)

    def lock_exclusive(file: IO[Any]) -> None:
        """
        Acquire an exclusive (write) lock via :func:`msvcrt.locking`.

        Parameters
        ----------
        file
            The file object to lock.
        """

        _prepare(file)
        msvcrt.locking(file.fileno(), msvcrt.LK_LOCK, _LOCK_NBYTES)

    def unlock(file: IO[Any]) -> None:
        """
        Release the lock via :func:`msvcrt.locking`.

        Parameters
        ----------
        file
            The file object to unlock.
        """

        file.seek(0)
        msvcrt.locking(file.fileno(), msvcrt.LK_UNLCK, _LOCK_NBYTES)

else:
    import fcntl

    def lock_shared(file: IO[Any]) -> None:
        """
        Acquire a shared (read) lock via :func:`fcntl.flock`.

        Parameters
        ----------
        file
            The file object to lock.
        """

        fcntl.flock(file, fcntl.LOCK_SH)

    def lock_exclusive(file: IO[Any]) -> None:
        """
        Acquire an exclusive (write) lock via :func:`fcntl.flock`.

        Parameters
        ----------
        file
            The file object to lock.
        """

        fcntl.flock(file, fcntl.LOCK_EX)

    def unlock(file: IO[Any]) -> None:
        """
        Release the lock via :func:`fcntl.flock`.

        Parameters
        ----------
        file
            The file object to unlock.
        """

        fcntl.flock(file, fcntl.LOCK_UN)
