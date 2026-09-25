"""
publisher_demo.py — standalone publisher process.

Run in one terminal:
    python publisher_demo.py

Then start subscriber_demo.py in another terminal (order does not matter).

The publisher creates the shared-memory segment, writes a simulated robot
state at 100 Hz, and reads back whatever velocity command the subscriber posts.
"""

import math
import sys
import time

from shmbridge import RobotState, ShmPublisher

SHM_NAME = "/sb_demo"
N_STEPS = 500
HZ = 100
DT = 1.0 / HZ


def main() -> None:
    pub = ShmPublisher(SHM_NAME, n_robots=1, n_consumers=1, heartbeat_every=1)
    pub.open()
    print(f"[pub] segment '{SHM_NAME}' open — publishing at {HZ} Hz for {N_STEPS} steps")

    try:
        step = 0
        while step < N_STEPS:
            t = step * DT
            state = RobotState()
            state.x = math.cos(2 * math.pi * t / 5.0)
            state.y = math.sin(2 * math.pi * t / 5.0)
            state.heading = 2 * math.pi * t / 5.0
            state.step = step
            state.sim_time = t
            pub.write_state(0, state)

            cmd = pub.read_best_cmd(0)
            if cmd is not None and step % 50 == 0:
                print(
                    f"[pub]  step={step:4d}  cmd linear={cmd.linear:.2f}"
                    f"  angular={cmd.angular:.2f}  seq={cmd.seq}"
                )

            step += 1
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\n[pub] interrupted")
    finally:
        pub.close()
        print("[pub] done — segment destroyed")


if __name__ == "__main__":
    sys.exit(main())
