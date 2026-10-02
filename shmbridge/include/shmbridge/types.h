/*
 * shmbridge/types.h  -  shared-memory layout v2 for the shmbridge library.
 *
 * Wire format improvements over shmbridge_v1:
 *
 *   ShmHeader  : magic (0x53484D42) + schema_version (2) + n_robots
 *                  + n_consumers at offsets 8-17 (were padding in v1)
 *   ShmStateSlot: writer_ts_ns at offset 88 (was _fill in v1)
 *   ShmCmdSlot  : writer_ts_ns at offset 32 (was _fill in v1)
 *
 * A v1 C++ controller can read a v2 segment without recompilation:
 *   - header.ready is still at offset 0 (uint64, value 1)
 *   - state/cmd payloads and seqlock positions are unchanged
 *   - new fields fall in bytes that v1 treated as padding
 *
 * Multi-robot, multi-consumer layout (N robots, C consumers each):
 *   offset 0          : ShmHeader       (128 B)
 *   offset 128        : ShmStateSlot[N] (N * 128 B)
 *   offset 128+128*N  : ShmCmdSlot[N*C] (N*C * 128 B)
 *   total             : 128 + 128*N + 128*N*C bytes
 *
 *   When C==1 the layout is byte-for-byte identical to the original v2.
 *   Cmd slot for robot r, consumer c: index = r * n_consumers + c.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#define SHMBRIDGE_MAGIC         0x53484D42u   /* ASCII "SHMB" */
#define SHMBRIDGE_VERSION       2
#define SHMBRIDGE_SHM_NAME      "/shmbridge_v2"

/* Size for N robots and C consumers per robot (raw, not page-aligned). */
#define SHMBRIDGE_SHM_SIZE_NC(n, c) \
    (128u + 128u * (unsigned)(n) + 128u * (unsigned)(n) * (unsigned)(c))

/* Legacy single-consumer alias; page-aligned variant. */
#define SHMBRIDGE_SHM_SIZE_N(n) SHMBRIDGE_SHM_SIZE_NC((n), 1)

/* Size of a segment for n_robots, page-aligned to 4096. */
#define SHMBRIDGE_SHM_SIZE_ALIGNED(n) \
    (( SHMBRIDGE_SHM_SIZE_N(n) + 4095u ) & ~4095u)

/* ── Robot state (72 bytes) — unchanged from v1 ─────────────────────────── */
typedef struct {
    double   x, y, heading;      /* world-frame pose  (m, m, rad)          */
    float    vx, vy, omega;      /* world-frame velocity (m/s, rad/s)       */
    float    goal_x, goal_y;     /* current goal (m)                        */
    float    goal_dist;          /* Euclidean distance to goal (m)           */
    uint64_t step;               /* sim step counter                        */
    double   sim_time;           /* simulated time (s)                      */
    uint8_t  reached;            /* 1 = goal reached                        */
    uint8_t  collision;          /* 1 = in collision                         */
    uint8_t  _pad[6];
} ShmState; /* sizeof == 72 */

/* ── Velocity command (16 bytes) — unchanged from v1 ───────────────────── */
typedef struct {
    float    linear;             /* forward velocity (m/s)                  */
    float    angular;            /* angular velocity (rad/s, CCW+)          */
    uint32_t seq;                /* command counter (monotone)              */
    uint32_t valid;              /* nonzero = fresh command                 */
} ShmCmd; /* sizeof == 16 */

/* ── State seqlock slot (128 bytes) ────────────────────────────────────── */
typedef struct {
    volatile uint64_t seq;           /* offset  0  odd while writing        */
    ShmState        state;         /* offset  8  72 bytes                 */
    volatile uint64_t seq2;          /* offset 80  mirrors seq when valid   */
    volatile uint64_t writer_ts_ns;  /* offset 88  monotonic ns (NEW in v2) */
    uint8_t           _fill[32];     /* offset 96  pad to 128               */
} ShmStateSlot; /* sizeof == 128 */

/* ── Cmd seqlock slot (128 bytes) ──────────────────────────────────────── */
typedef struct {
    volatile uint64_t seq;           /* offset  0                           */
    ShmCmd          cmd;           /* offset  8  16 bytes                 */
    volatile uint64_t seq2;          /* offset 24                           */
    volatile uint64_t writer_ts_ns;  /* offset 32  monotonic ns (NEW in v2) */
    uint8_t           _fill[88];     /* offset 40  pad to 128               */
} ShmCmdSlot; /* sizeof == 128 */

/* ── Shared-memory header (128 bytes) ──────────────────────────────────── */
/*
 * ext_schema_version (offset 20) versions ONLY the extended block's own
 * layout (see ext_core.hpp's ShmExtBlock) -- independent of schema_version
 * above, which is the base state/cmd protocol every ShmBridge/ShmPublisher/
 * ShmSubscriber segment uses. A plain base segment never sets it (stays 0,
 * ignored). It's carved out of what used to be _fill padding (offsets
 * 18-127 were always zero before), so ready/magic/schema_version/n_robots/
 * n_consumers keep their exact original offsets -- a pre-existing base-only
 * reader that has never heard of ext_schema_version silently ignores it,
 * same as it always ignored this padding. (2 bytes of implicit compiler
 * padding sit between n_consumers at offset 17 and this uint32_t, placing
 * it at offset 20 to keep it 4-byte aligned -- verified byte-for-byte
 * identical to _types.py's ctypes _ShmHeader.)
 */
typedef struct {
    volatile uint64_t ready;         /* offset  0  1 = initialized (v1 compat) */
    uint32_t          magic;         /* offset  8  SHMBRIDGE_MAGIC             */
    uint32_t          schema_version;/* offset 12  SHMBRIDGE_VERSION           */
    uint8_t           n_robots;      /* offset 16  number of robot slots       */
    uint8_t           n_consumers;   /* offset 17  cmd writers per robot (NEW) */
    uint32_t          ext_schema_version; /* offset 20  see comment above (NEW) */
    uint8_t           _fill[104];    /* offset 24  pad to 128                  */
} ShmHeader; /* sizeof == 128 */

/* Single-robot, single-consumer convenience block. */
typedef struct {
    ShmHeader    header;
    ShmStateSlot states[1];
    ShmCmdSlot   cmds[1];  /* index = robot * n_consumers + consumer */
} ShmBlock; /* sizeof == 384 for n=1, c=1 */

/*
 * Cmd-slot accessor for multi-consumer segments.
 * blk must be cast to (char *) to do pointer arithmetic past cmds[0].
 *
 * Example (n_consumers known at runtime from header):
 *   uint8_t nc = blk->header.n_consumers;
 *   ShmCmdSlot *slot = SHMBRIDGE_CMD(blk, robot_idx, consumer_idx, nc);
 */
#define SHMBRIDGE_CMD(blk, robot, consumer, n_consumers) \
    (&(blk)->cmds[(unsigned)(robot) * (unsigned)(n_consumers) + (unsigned)(consumer)])
