"""
subscriber_demo.py — standalone subscriber process with publisher-restart support.

Run in a second terminal alongside publisher_demo.py (either order; either can
restart any number of times):

    python subscriber_demo.py       # terminal 1 — stays alive indefinitely
    python publisher_demo.py        # terminal 2 — run once, exit, run again...

Lifecycle handled transparently:
  - publisher not up yet   → subscriber waits (up to ATTACH_TIMEOUT_MS)
  - publisher exits        → subscriber detects staleness, loops, re-attaches
  - publisher restarts     → subscriber re-attaches to the new segment
"""

import math
import sys
import time

from shmbridge import ShmSubscriber

SHM_NAME = "/sb_demo"
HZ = 100
DT = 1.0 / HZ
ATTACH_TIMEOUT_MS = 30_000  # wait up to 30 s for publisher to (re-)appear


def main() -> None:
    sub = ShmSubscriber(SHM_NAME, n_robots=1)
    consumer_idx = 0
    seq = 0

    try:
        while True:
            # ── wait for (next) publisher ──────────────────────────────────
            print(
                f"[sub] waiting for publisher on '{SHM_NAME}'"
                f" (up to {ATTACH_TIMEOUT_MS // 1000} s)…"
            )
            try:
                # attach() auto-detaches any previous mapping first, so calling
                # it here in a loop is safe after a publisher-restart.
                sub.attach(timeout_ms=ATTACH_TIMEOUT_MS)
            except TimeoutError:
                print("[sub] timed out — no publisher appeared, exiting")
                break

            print("[sub] attached — reading state and writing commands")

            # ── read loop: runs until publisher goes away ──────────────────
            while True:
                state = sub.read_state_spin(0)
                if state is not None:
                    err_x = -state.x
                    err_y = -state.y
                    dist = math.hypot(err_x, err_y)
                    desired = math.atan2(err_y, err_x)
                    heading_err = desired - state.heading

                    # Normalise to (-pi, pi]
                    while heading_err > math.pi:
                        heading_err -= 2 * math.pi
                    while heading_err < -math.pi:
                        heading_err += 2 * math.pi

                    linear = min(0.5 * dist, 1.0)
                    angular = 1.5 * heading_err
                    sub.write_cmd(0, consumer_idx, linear, angular)
                    seq += 1

                    if seq % 50 == 0:
                        print(
                            f"[sub]  x={state.x:+.3f}  y={state.y:+.3f}"
                            f"  step={state.step}"
                            f"  → lin={linear:.2f}  ang={angular:.2f}"
                        )

                if not sub.is_publisher_alive(max_age_ms=500):
                    print("[sub] publisher went away — waiting for restart…")
                    sub.detach()
                    break

                time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[sub] interrupted")
    finally:
        sub.detach()
        print("[sub] detached")


if __name__ == "__main__":
    sys.exit(main())
