import time
import atexit
import os
import functools
import tqdm
import threading


@functools.lru_cache(maxsize=1)
def is_debugging():
    return os.getenv("DEBUG") is not None


@functools.lru_cache(maxsize=1)
def is_debugging_all():
    return os.getenv("DEBUG") == "all"


@functools.lru_cache(maxsize=1)
def is_profiling():
    return os.getenv("PROFILING") is not None


@functools.lru_cache(maxsize=1)
def is_profiling_all():
    return os.getenv("PROFILING") == "all"


class DurationProgress:
    def __init__(self, *args, **kwargs):
        self.pbar = tqdm.tqdm(
            *args, **kwargs, leave=False, bar_format="{desc}: {n_fmt}s/{total_fmt}s (may exceed estimated time)"
        )
        self.thread = None
        self.has_finished = False

    def __enter__(self):
        self.thread = threading.Thread(target=self.__update_progress)
        self.thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.has_finished = True
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()
        if self.pbar is not None:
            self.pbar.close()

    def __update_progress(self):
        while not self.has_finished:
            self.pbar.update()
            time.sleep(1)


def profiling(func, prefix=""):
    stats = {"count": 0, "first_call_time": None, "duration": 0}

    def flush():
        freq = stats["count"] / (time.time() - stats["first_call_time"])
        mean_duration = stats["duration"] / stats["count"]
        print(f"[{prefix + func.__name__}]\tcount: {stats['count']}\tfreq: {freq:.2f}/s\tduration: ~{mean_duration / 1e-3:.2f}ms")

    def wrapper(*args, **kwargs):
        if not is_profiling():
            return func(*args, **kwargs)

        # register flush function to be called at exit
        if stats["first_call_time"] is None:
            stats["first_call_time"] = time.time()
            atexit.register(flush)

        stats["count"] += 1
        t0 = time.time()
        res = func(*args, **kwargs)
        t1 = time.time()
        stats["duration"] += t1 - t0
        return res

    return wrapper


class ProfilingMeta(type):
    def __new__(cls, name, bases, attrs):
        for key, value in attrs.items():
            if callable(value) and (not "__" in key or is_profiling_all()):
                attrs[key] = profiling(value, prefix=f"{name}.")
        return super(ProfilingMeta, cls).__new__(cls, name, bases, attrs)
