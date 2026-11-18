"""A small sampling profiler for every thread of a process (debugging aid, off unless PY_SAMPLE is set).

    PY_SAMPLE=out/radar_array.samples ./run_sitl.sh ...      (radar_array.py starts it)

Every INTERVAL s it records each running thread's stack; every DUMP s it writes, per function, the share
of samples in which it is on top of a stack (self) and anywhere in it (total). Threads waiting in
known blocking calls are counted separately as idle.
"""

import collections
import os
import sys
import threading
import time
import traceback

INTERVAL = 0.005
DUMP = 20.0
IDLE = ("wait", "sleep", "select", "poll", "recv", "accept", "spin_once", "spin", "_wait_for_tstate_lock", "acquire")


def start(path):
    me = threading.get_ident()
    top, anywhere = collections.Counter(), collections.Counter()
    count = {"samples": 0, "idle": 0}

    def run():
        last = time.monotonic()
        while True:
            time.sleep(INTERVAL)
            for ident, frame in sys._current_frames().items():
                if ident == me_thread[0]:
                    continue
                stack = traceback.extract_stack(frame)
                if not stack:
                    continue
                name = lambda f: f"{os.path.basename(f.filename)}:{f.name}"  # noqa: E731
                count["samples"] += 1
                if stack[-1].name in IDLE:
                    count["idle"] += 1
                    continue
                top[name(stack[-1])] += 1
                for f in {name(f) for f in stack}:
                    anywhere[f] += 1
            if time.monotonic() - last > DUMP:
                last = time.monotonic()
                n = max(count["samples"], 1)
                with open(path, "w") as out:
                    out.write(f"{count['samples']} thread samples, {100 * count['idle'] / n:.0f} % idle\n\nself:\n")
                    for k, v in top.most_common(25):
                        out.write(f"  {100 * v / n:5.1f} %  {k}\n")
                    out.write("\ntotal:\n")
                    for k, v in anywhere.most_common(30):
                        out.write(f"  {100 * v / n:5.1f} %  {k}\n")

    me_thread = [None]
    t = threading.Thread(target=run, daemon=True)
    t.start()
    me_thread[0] = t.ident
