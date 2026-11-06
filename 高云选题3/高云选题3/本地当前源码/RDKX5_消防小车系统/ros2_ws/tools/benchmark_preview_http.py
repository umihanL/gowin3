#!/usr/bin/env python3
"""Measure HTTP small-response stalls in isolated localhost preview servers.

Uses no ROS, camera, inference, hardware, or live preview service. The alternating
runs override only the socket option on temporary handler classes. Every response
must retain exactly the same bytes. Results measure unchanged-poll round trips,
not image decoding, real network latency, or YOLO throughput.
"""
import argparse
import hashlib
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
import math
import platform
import statistics
import threading
import time

from detection_preview import PreviewState, handler_class


def run(nodelay, count, warmup):
    handler = handler_class(PreviewState(), b"preview-page")
    handler.disable_nagle_algorithm = nodelay
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = HTTPConnection("127.0.0.1", server.server_port, timeout=5)
    timings, digests = [], set()
    try:
        for index in range(count + warmup):
            started = time.perf_counter()
            client.request("GET", "/api/frame?after=unchanged")
            response = client.getresponse()
            body = response.read()
            elapsed = (time.perf_counter() - started) * 1000
            if response.status != 200:
                raise RuntimeError(f"HTTP {response.status}")
            digests.add(hashlib.sha256(body).hexdigest())
            if index >= warmup:
                timings.append(elapsed)
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
    if len(digests) != 1:
        raise AssertionError("Unchanged response bytes unexpectedly varied")
    return dict(nodelay=nodelay, n=len(timings), median_ms=statistics.median(timings),
                mean_ms=statistics.mean(timings),
                p95_ms=sorted(timings)[math.ceil(.95 * len(timings)) - 1],
                max_ms=max(timings), response_sha256=next(iter(digests)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=50)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--cycles", type=int, default=2)
    args = parser.parse_args()
    if args.iterations < 1 or args.warmup < 0 or args.cycles < 1:
        parser.error("iterations/cycles must be positive; warmup must be nonnegative")
    runs = [run(nodelay, args.iterations, args.warmup)
            for _ in range(args.cycles) for nodelay in (False, True)]
    if len({result["response_sha256"] for result in runs}) != 1:
        raise AssertionError("Socket variants changed response bytes")
    print(json.dumps(dict(platform=platform.platform(), identical_wire_bytes=True,
                          metric="unchanged_preview_poll_roundtrip", runs=runs), indent=2))


if __name__ == "__main__":
    main()
