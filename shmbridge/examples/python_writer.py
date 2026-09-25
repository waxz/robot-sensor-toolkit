"""
python_writer.py — publish robot state and read back velocity commands.

Pairs with controller.cpp (or any ShmSubscriber-based controller).
Run either in any order — the controller waits for the publisher to appear.

Usage:
    pip install -e ../../   # install shmbridge package
    python python_writer.py
"""

import time

from shmbridge import RobotState, ShmPublisher

try:
    from shmbridge._core import LoopSleeper  # precise 100 Hz timing
    _HAS_LOOP_SLEEPER = True
except ImportError:
    _HAS_LOOP_SLEEPER = False

SHM_NAME = "/shmbridge_v2"
N_STEPS  = 500
GOAL_X   = 5.0
GOAL_Y   = 0.0


def main() -> None:
    pub = ShmPublisher(SHM_NAME, n_robots=1, n_consumers=1, heartbeat_every=1)
    pub.open()
    print(f"[pub] segment '{SHM_NAME}' open — writing at 100 Hz")
    print("[pub] waiting for C++ controller to attach …")

    if _HAS_LOOP_SLEEPER:
        sleeper = LoopSleeper(100.0)
        print("[pub] using LoopSleeper for precise 100 Hz timing")

    try:
        for step in range(N_STEPS):
            if _HAS_LOOP_SLEEPER:
                sleeper.start()

            t = step * 0.01
            s = RobotState()
            s.x         = 0.1 * step
            s.y         = 0.0
            s.heading   = 0.0
            s.goal_x    = GOAL_X
            s.goal_y    = GOAL_Y
            s.goal_dist = max(0.0, GOAL_X - 0.1 * step)
            s.step      = step
            s.sim_time  = t
            pub.write_state(0, s)

            cmd = pub.read_best_cmd(0)
            if step % 50 == 0:
                if cmd is not None:
                    print(
                        f"[pub] step={step:5d}  cmd"
                        f" lin={cmd.linear:.3f}  ang={cmd.angular:.3f}"
                        f"  seq={cmd.seq}"
                    )
                else:
                    print(f"[pub] step={step:5d}  no cmd yet")

            if not pub.is_controller_alive(max_age_ms=200):
                print("[pub] WARNING: controller stale (> 200 ms)")

            if _HAS_LOOP_SLEEPER:
                sleeper.sleep()
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[pub] interrupted")
    finally:
        pub.close()
        print("[pub] segment closed")


if __name__ == "__main__":
    main()
