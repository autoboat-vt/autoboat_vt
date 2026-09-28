#!/usr/bin/env python3
"""
Benchmark JPEG compression latency, throughput, and bandwidth consumption for 360p images at 5 Hz.

This benchmark evaluates OpenCV's `cv2.imencode('.jpg', ...)` (backed by libjpeg-turbo)
under both real-time paced conditions (simulating a 5 Hz camera/telemetry stream)
and tight-loop throughput conditions.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import platform
import shutil
import subprocess
import time
from typing import Any

import cv2
import numpy as np

try:
    from PIL import Image

    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False


def get_cpu_info() -> str:
    """Detect and return CPU model information."""
    if platform.system() == "Linux":
        try:
            with open("/proc/cpuinfo", "r", encoding="utf-8") as f:
                for line in f:
                    if "model name" in line:
                        return line.split(":", 1)[1].strip()
        except OSError:
            pass
        lscpu_bin = shutil.which("lscpu")
        if lscpu_bin:
            try:
                out = subprocess.check_output([lscpu_bin], text=True)
                for line in out.splitlines():
                    if "Model name:" in line:
                        return line.split(":", 1)[1].strip()
            except (subprocess.SubprocessError, OSError):
                pass
    return platform.processor() or "Unknown CPU"


def get_opencv_jpeg_backend() -> str:
    """Retrieve OpenCV JPEG backend build information."""
    build_info = cv2.getBuildInformation()
    for line in build_info.splitlines():
        if "JPEG:" in line and "JPEG 2000" not in line:
            return line.strip()
    return "Unknown JPEG backend"


def create_synthetic_scene(width: int, height: int) -> np.ndarray:
    """
    Generate a realistic synthetic maritime/outdoor scene (sky, water horizon, and shapes).

    This provides realistic spatial redundancy and DCT coefficients representative of
    real-world boat camera views, rather than pure white noise.
    """
    rng = np.random.default_rng(seed=42)
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Sky gradient (top half)
    horizon = int(height * 0.45)
    for y in range(horizon):
        alpha = y / max(1, horizon)
        # BGR: Sky blue gradient (light blue to deep blue)
        b = int(220 - 40 * alpha)
        g = int(180 - 50 * alpha)
        r = int(120 - 40 * alpha)
        img[y, :] = (b, g, r)

    # Water gradient (bottom half)
    for y in range(horizon, height):
        alpha = (y - horizon) / max(1, height - horizon)
        # BGR: Water dark cyan / navy gradient
        b = int(140 - 60 * alpha)
        g = int(100 - 40 * alpha)
        r = int(30 - 15 * alpha)
        img[y, :] = (b, g, r)

    # Add boat / buoy shapes to create realistic edge frequencies
    cv2.rectangle(img, (int(width * 0.4), int(horizon - 25)), (int(width * 0.6), int(horizon + 20)), (40, 40, 200), -1)
    cv2.circle(img, (int(width * 0.5), int(horizon - 35)), 15, (0, 215, 255), -1)
    cv2.line(img, (0, horizon), (width, horizon), (180, 180, 180), 1)

    # Slight Gaussian noise for sensor simulation
    noise = rng.normal(0, 3, img.shape).astype(np.int16)
    return np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def load_or_create_image(image_path: str | None, width: int, height: int) -> tuple[np.ndarray, str]:
    """Load specified image or fall back to repository test image or synthetic scene."""
    candidate_paths = []
    if image_path:
        candidate_paths.append(image_path)

    # Default candidate paths in autoboat_vt repo
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    candidate_paths.append(os.path.join(repo_root, "ground_station/app_data/git_keep/assets/test.jpg"))

    for path in candidate_paths:
        if os.path.isfile(path):
            img = cv2.imread(path)
            if img is not None:
                resized = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
                return resized, f"File: {os.path.basename(path)} (resized to {width}x{height})"

    synth = create_synthetic_scene(width, height)
    return synth, f"Synthetic Maritime Scene ({width}x{height})"


def compute_statistics(values: list[float]) -> dict[str, float]:
    """Compute summary statistics for a list of metric measurements."""
    if not values:
        return {}
    arr = np.array(values, dtype=np.float64)
    return {
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "median": float(np.median(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "p25": float(np.percentile(arr, 25)),
        "p75": float(np.percentile(arr, 75)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
    }


def benchmark_opencv_tight_loop(
    img: np.ndarray, quality: int, iterations: int = 300, warmup: int = 20
) -> tuple[dict[str, float], int]:
    """Run OpenCV tight-loop benchmark without pacing/sleep overhead."""
    # Warmup
    for _ in range(warmup):
        _, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])

    durations_ms: list[float] = []
    last_size = len(buf)

    for _ in range(iterations):
        t0 = time.perf_counter()
        _, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        t1 = time.perf_counter()
        durations_ms.append((t1 - t0) * 1000.0)
        last_size = len(buf)

    return compute_statistics(durations_ms), last_size


def benchmark_pillow_tight_loop(
    img: np.ndarray, quality: int, iterations: int = 150, warmup: int = 10
) -> tuple[dict[str, float], int]:
    """Run Pillow tight-loop benchmark."""
    if not PIL_AVAILABLE:
        return {}, 0

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)

    # Warmup
    for _ in range(warmup):
        out = io.BytesIO()
        pil_img.save(out, format="JPEG", quality=quality)

    durations_ms: list[float] = []
    last_size = 0

    for _ in range(iterations):
        out = io.BytesIO()
        t0 = time.perf_counter()
        pil_img.save(out, format="JPEG", quality=quality)
        t1 = time.perf_counter()
        durations_ms.append((t1 - t0) * 1000.0)
        last_size = out.tell()

    return compute_statistics(durations_ms), last_size


def benchmark_paced_5hz(
    img: np.ndarray,
    quality: int,
    rate_hz: float = 5.0,
    duration_sec: float = 10.0,
) -> dict[str, Any]:
    """
    Simulate real-time 5 Hz streaming loop, measuring compression time and timing budget.

    Period T = 1.0 / rate_hz (200 ms for 5 Hz).
    """
    target_period = 1.0 / rate_hz
    total_frames = int(rate_hz * duration_sec)

    # Warmup
    for _ in range(5):
        cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])

    compression_times_ms: list[float] = []
    loop_intervals_ms: list[float] = []
    sleep_times_ms: list[float] = []
    payload_sizes: list[int] = []

    t_prev = time.perf_counter()
    next_deadline = t_prev + target_period

    for _ in range(total_frames):
        t_start = time.perf_counter()
        _, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        t_comp = time.perf_counter()

        comp_ms = (t_comp - t_start) * 1000.0
        compression_times_ms.append(comp_ms)
        payload_sizes.append(len(buf))

        now = time.perf_counter()
        sleep_dur = max(0.0, next_deadline - now)
        sleep_times_ms.append(sleep_dur * 1000.0)

        if sleep_dur > 0:
            time.sleep(sleep_dur)

        t_now = time.perf_counter()
        loop_intervals_ms.append((t_now - t_prev) * 1000.0)
        t_prev = t_now
        next_deadline += target_period

    # Ignore first loop interval due to startup alignment
    steady_intervals = loop_intervals_ms[1:] if len(loop_intervals_ms) > 1 else loop_intervals_ms

    return {
        "target_rate_hz": rate_hz,
        "target_period_ms": target_period * 1000.0,
        "total_frames": total_frames,
        "duration_sec": duration_sec,
        "compression_stats": compute_statistics(compression_times_ms),
        "loop_interval_stats": compute_statistics(steady_intervals),
        "sleep_stats": compute_statistics(sleep_times_ms),
        "mean_payload_bytes": float(np.mean(payload_sizes)),
        "effective_fps": 1000.0 / float(np.mean(steady_intervals)) if steady_intervals else 0.0,
    }


def format_table(headers: list[str], rows: list[list[str]]) -> str:
    """Format an ASCII / Markdown compatible table."""
    widths = [len(h) for h in headers]
    for row in rows:
        for idx, cell in enumerate(row):
            widths[idx] = max(widths[idx], len(cell))

    header_line = " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
    separator_line = "-+-".join("-" * widths[i] for i in range(len(headers)))
    row_lines = [" | ".join(cell.rjust(widths[i]) for i, cell in enumerate(row)) for row in rows]
    return f"{header_line}\n{separator_line}\n" + "\n".join(row_lines)


def run_benchmark(args: argparse.Namespace) -> None:
    """Execute all benchmarks and format results."""
    width, height = (int(x) for x in args.resolution.lower().split("x"))
    raw_frame_bytes = width * height * 3

    img, img_description = load_or_create_image(args.image, width, height)
    cpu_info = get_cpu_info()
    backend_info = get_opencv_jpeg_backend()

    print("=" * 80)
    print("  JPEG COMPRESSION BENCHMARK: 360p IMAGE AT 5 Hz")
    print("=" * 80)
    print(f"  CPU Model:             {cpu_info}")
    print(f"  OpenCV Backend:        {backend_info}")
    print(f"  Image Source:          {img_description}")
    print(f"  Resolution:            {width}x{height} pixels ({raw_frame_bytes / 1024:.1f} KB uncompressed BGR)")
    print(f"  Target Rate:           {args.rate:.1f} Hz (Period = {1000.0 / args.rate:.1f} ms)")
    print("=" * 80)
    print()

    # ---------------------------------------------------------
    # 1. Paced 5 Hz Real-Time Test
    # ---------------------------------------------------------
    print(f"[1/4] Running Paced Real-Time Benchmark ({args.rate:.1f} Hz for {args.duration:.1f}s, Quality={args.quality})...")
    paced_results = benchmark_paced_5hz(img, args.quality, rate_hz=args.rate, duration_sec=args.duration)
    c_stats = paced_results["compression_stats"]
    l_stats = paced_results["loop_interval_stats"]
    payload_kb = paced_results["mean_payload_bytes"] / 1024.0
    bandwidth_kbps = (paced_results["mean_payload_bytes"] * 8.0 * args.rate) / 1000.0
    budget_pct = (c_stats["mean"] / paced_results["target_period_ms"]) * 100.0
    comp_ratio = raw_frame_bytes / paced_results["mean_payload_bytes"]

    print("\n--- PACED 5 Hz STREAMING METRICS ---")
    print(f"  Effective Frame Rate:    {paced_results['effective_fps']:.2f} FPS (Target: {args.rate:.1f} FPS)")
    print(f"  Loop Period (Target):    {paced_results['target_period_ms']:.2f} ms")
    print(f"  Actual Loop Period:      {l_stats['mean']:.2f} ms +/- {l_stats['std']:.2f} ms")
    print(f"  JPEG Compression Time:   Mean: {c_stats['mean']:.3f} ms  |  Median: {c_stats['median']:.3f} ms")
    print(f"                           Min:  {c_stats['min']:.3f} ms  |  Max:    {c_stats['max']:.3f} ms")
    print(f"                           P95:  {c_stats['p95']:.3f} ms  |  P99:    {c_stats['p99']:.3f} ms")
    print(f"  200ms Budget Consumed:   {budget_pct:.2f}% (Idle Headroom: {100.0 - budget_pct:.2f}%)")
    print(f"  Encoded Frame Size:      {payload_kb:.2f} KB (Compression Ratio: {comp_ratio:.1f}:1)")
    print(
        f"  Telemetry Bandwidth:     {payload_kb * args.rate:.2f} KB/s  "
        f"({bandwidth_kbps:.1f} kbps / {bandwidth_kbps / 1000.0:.2f} Mbps)"
    )
    print()

    # ---------------------------------------------------------
    # 2. Tight-Loop Latency & Throughput Benchmark
    # ---------------------------------------------------------
    print(f"[2/4] Running Tight-Loop Latency Benchmark ({args.iterations} iterations, Quality={args.quality})...")
    tight_stats, tight_size = benchmark_opencv_tight_loop(img, args.quality, iterations=args.iterations)
    max_throughput_fps = 1000.0 / tight_stats["mean"] if tight_stats["mean"] > 0 else 0

    print("\n--- TIGHT-LOOP PEAK LATENCY & THROUGHPUT (Active CPU) ---")
    print(f"  Mean Latency:            {tight_stats['mean']:.3f} ms (+/- {tight_stats['std']:.3f} ms)")
    print(f"  Median Latency:          {tight_stats['median']:.3f} ms")
    print(f"  Min / Max:               {tight_stats['min']:.3f} ms / {tight_stats['max']:.3f} ms")
    print(f"  P25 / P75:               {tight_stats['p25']:.3f} ms / {tight_stats['p75']:.3f} ms")
    print(f"  P95 / P99:               {tight_stats['p95']:.3f} ms / {tight_stats['p99']:.3f} ms")
    print(f"  Max Theoretical Rate:    {max_throughput_fps:.1f} FPS ({max_throughput_fps / args.rate:.1f}x faster than 5 Hz)")
    print()

    # ---------------------------------------------------------
    # 3. Quality vs Latency vs Bandwidth Sweep
    # ---------------------------------------------------------
    print("[3/4] Running JPEG Quality Sweep (Qualities 30 to 95)...")
    qualities = [30, 50, 70, 75, 80, 85, 90, 95]
    sweep_headers = ["Quality", "Mean Latency", "P95 Latency", "Frame Size", "Ratio", "Bandwidth @ 5Hz", "200ms Budget"]
    sweep_rows = []
    sweep_data = []

    for q in qualities:
        q_stats, q_size = benchmark_opencv_tight_loop(img, q, iterations=120)
        q_kb = q_size / 1024.0
        q_kbps = (q_size * 8.0 * args.rate) / 1000.0
        q_budget = (q_stats["mean"] / 200.0) * 100.0
        ratio = raw_frame_bytes / max(1, q_size)
        mark = " (default)" if q == 50 else ""
        sweep_rows.append(
            [
                f"{q}{mark}",
                f"{q_stats['mean']:.3f} ms",
                f"{q_stats['p95']:.3f} ms",
                f"{q_kb:.1f} KB",
                f"{ratio:.1f}:1",
                f"{q_kbps:.1f} kbps",
                f"{q_budget:.2f}%",
            ]
        )
        sweep_data.append(
            {
                "quality": q,
                "mean_ms": q_stats["mean"],
                "p95_ms": q_stats["p95"],
                "size_kb": q_kb,
                "bandwidth_kbps": q_kbps,
                "budget_pct": q_budget,
            }
        )

    print("\n--- QUALITY SWEEP COMPARISON ---")
    print(format_table(sweep_headers, sweep_rows))
    print()

    # ---------------------------------------------------------
    # 4. Engine / Pattern Comparisons
    # ---------------------------------------------------------
    print("[4/4] Comparing Image Patterns & Libraries (Quality=50)...")
    comp_headers = ["Test Configuration", "Backend", "Mean Latency", "Median", "P95", "Frame Size"]
    comp_rows = []

    # OpenCV Realistic Image
    comp_rows.append(
        [
            "Realistic Scene",
            "OpenCV (libjpeg-turbo)",
            f"{tight_stats['mean']:.3f} ms",
            f"{tight_stats['median']:.3f} ms",
            f"{tight_stats['p95']:.3f} ms",
            f"{tight_size / 1024.0:.1f} KB",
        ]
    )

    # OpenCV Random Noise (Worst-Case)
    rng = np.random.default_rng(seed=123)
    noise_img = rng.integers(0, 256, (height, width, 3), dtype=np.uint8)
    n_stats, n_size = benchmark_opencv_tight_loop(noise_img, args.quality, iterations=80)
    comp_rows.append(
        [
            "Random Noise (Worst Case)",
            "OpenCV (libjpeg-turbo)",
            f"{n_stats['mean']:.3f} ms",
            f"{n_stats['median']:.3f} ms",
            f"{n_stats['p95']:.3f} ms",
            f"{n_size / 1024.0:.1f} KB",
        ]
    )

    # Pillow comparison
    if PIL_AVAILABLE:
        p_stats, p_size = benchmark_pillow_tight_loop(img, args.quality, iterations=80)
        comp_rows.append(
            [
                "Realistic Scene",
                "Pillow (PIL)",
                f"{p_stats['mean']:.3f} ms",
                f"{p_stats['median']:.3f} ms",
                f"{p_stats['p95']:.3f} ms",
                f"{p_size / 1024.0:.1f} KB",
            ]
        )

    print("\n--- IMAGE CONTENT & BACKEND COMPARISON ---")
    print(format_table(comp_headers, comp_rows))
    print()

    # Optional JSON output
    if args.save_json:
        result_payload = {
            "system": {
                "cpu": cpu_info,
                "opencv_backend": backend_info,
                "platform": platform.platform(),
            },
            "parameters": {
                "resolution": f"{width}x{height}",
                "rate_hz": args.rate,
                "quality": args.quality,
                "image_description": img_description,
            },
            "paced_5hz": paced_results,
            "tight_loop": tight_stats,
            "quality_sweep": sweep_data,
        }
        with open(args.save_json, "w", encoding="utf-8") as jf:
            json.dump(result_payload, jf, indent=2)
        print(f"Results saved to {args.save_json}")


def main() -> None:
    """Parse arguments and start benchmark."""
    parser = argparse.ArgumentParser(
        description="Benchmark JPEG compression latency for 360p images at 5 Hz.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--resolution",
        type=str,
        default="640x360",
        help="Image resolution WxH (e.g., 640x360 for 16:9 360p, or 480x360 for 4:3 360p)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Target streaming frequency in Hz",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Duration in seconds for the real-time paced 5 Hz benchmark",
    )
    parser.add_argument(
        "--quality",
        type=int,
        default=50,
        help="JPEG quality factor (1-100), default 50 matches cv_default_parameters.jsonc",
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=300,
        help="Number of iterations for tight-loop latency test",
    )
    parser.add_argument(
        "--image",
        type=str,
        default=None,
        help="Optional path to custom test image (falls back to repo test image or synthetic scene)",
    )
    parser.add_argument(
        "--save-json",
        type=str,
        default=None,
        help="Optional path to export JSON results",
    )

    args = parser.parse_args()
    run_benchmark(args)


if __name__ == "__main__":
    main()
