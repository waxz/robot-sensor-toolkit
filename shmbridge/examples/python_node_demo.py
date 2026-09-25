"""
python_node_demo.py — in-process publisher + subscriber demo.

Demonstrates that publisher and subscriber are lifecycle-independent:
  • Subscriber starts first; attach() retries until publisher opens.
  • Publisher runs for N_STEPS, shuts down, sleeps 1 s, then restarts.
  • Subscriber detects the gap and re-attaches to the new segment automatically.

For a real multi-process setup see:
    publisher_demo.py   — run in one terminal
    subscriber_demo.py  — run in a second terminal (either order; publisher can
                          stop and restart while subscriber keeps running)

Usage:
    pip install -e ../..
    python python_node_demo.py
"""

import math
import threading
import time

from shmbridge import RobotState, ShmPublisher, ShmSubscriber

SHM_NAME = "/sb_demo"
N_STEPS = 250        # steps per publisher run
N_RUNS = 2           # publisher restarts this many times
RESTART_PAUSE_S = 1  # gap between publisher runs
HZ = 100
DT = 1.0 / HZ


def run_publisher(stop: threading.Event) -> None:
    for run in range(N_RUNS):
        if stop.is_set():
            break
        with ShmPublisher(SHM_NAME, n_robots=1, n_consumers=1, heartbeat_every=1) as pub:
            print(f"[pub]  run {run + 1}/{N_RUNS} — segment open, publishing at {HZ} Hz")
            step = 0
            while not stop.is_set() and step < N_STEPS:
                t = step * DT
                state = RobotState()
                state.x = math.cos(2 * math.pi * t / 5.0)
                state.y = math.sin(2 * math.pi * t / 5.0)
                state.heading = 2 * math.pi * t / 5.0
                state.step = run * N_STEPS + step
                state.sim_time = t
                pub.write_state(0, state)

                cmd = pub.read_best_cmd(0)
                if cmd is not None and step % 50 == 0:
                    print(
                        f"[pub]  step={state.step:4d}  cmd"
                        f" lin={cmd.linear:.2f} ang={cmd.angular:.2f}"
                    )
                step += 1
                time.sleep(DT)
            print(f"[pub]  run {run + 1}/{N_RUNS} done — shutting down")
        # segment is now unlinked (ShmPublisher.__exit__ called close())

        if run < N_RUNS - 1 and not stop.is_set():
            print(f"[pub]  pausing {RESTART_PAUSE_S} s before restart…")
            time.sleep(RESTART_PAUSE_S)

    print("[pub]  all runs complete")


def run_subscriber(stop: threading.Event) -> None:
    sub = ShmSubscriber(SHM_NAME, n_robots=1)
    consumer_idx = 0
    seq = 0

    while not stop.is_set():
        # attach() retries until the publisher creates the segment, and
        # auto-detaches any previous mapping so re-calling it is safe.
        print("[sub]  waiting for publisher…")
        try:
            sub.attach(timeout_ms=5000)
        except TimeoutError:
            print("[sub]  no publisher within 5 s — stopping")
            break

        print("[sub]  attached — reading state")

        while not stop.is_set():
            state = sub.read_state_spin(0)
            if state is not None:
                err_x = -state.x
                err_y = -state.y
                dist = math.hypot(err_x, err_y)
                desired = math.atan2(err_y, err_x)
                heading_err = desired - state.heading
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
                print("[sub]  publisher went away — will reconnect…")
                sub.detach()
                break

            time.sleep(DT)

    sub.detach()
    print("[sub]  stopped")


def main() -> None:
    stop = threading.Event()

    # Start subscriber first — demonstrates ordering independence
    sub_thread = threading.Thread(target=run_subscriber, args=(stop,), daemon=True)
    pub_thread = threading.Thread(target=run_publisher, args=(stop,), daemon=True)

    sub_thread.start()
    pub_thread.start()

    try:
        pub_thread.join()
    except KeyboardInterrupt:
        print("\nInterrupted — shutting down")
    finally:
        stop.set()
        sub_thread.join(timeout=3.0)
        print("Done.")


if __name__ == "__main__":
    main()
