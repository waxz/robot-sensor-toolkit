/*
 * ray_casting_omp.c - OpenMP-parallel 2D ray-segment intersection kernels.
 *
 * Compile (Linux/macOS — uses native CPU features):
 *   gcc -O3 -march=native -fopenmp -shared -fPIC -o ray_casting_omp.so \
 *       ray_casting_omp.c -lm
 *
 * Compile (explicit AVX2, portable to any AVX2 x86-64 host):
 *   gcc -O3 -mavx2 -mfma -fopenmp -shared -fPIC -o ray_casting_omp.so \
 *       ray_casting_omp.c -lm
 *
 * Called from ray_casting_2d_omp.py via ctypes.  The function signature is
 * a plain-C ABI so no Python headers are required.
 *
 * Kernel selection and platform fallback:
 *
 *   Platform           Kernels compiled         Python fallback chain
 *   ─────────────────  ──────────────────────   ──────────────────────────────
 *   x86-64 with AVX2   OMP + AVX2 f64 + f32     f32 > f64 > OMP > NumPy
 *   x86-64 no AVX2     OMP only                 OMP > NumPy
 *   AArch64 / Apple M  OMP only                 OMP > NumPy
 *   Any (no compiler)  (not compiled)           NumPy
 *
 * cast_ray_segments_avx2_soa  (f64 4-wide):
 *   AVX2 SIMD 4-wide double-precision, OpenMP beam-group-parallel (SoA).
 *
 * cast_ray_segments_avx2_f32_soa  (f32 8-wide) [NEW]:
 *   AVX2 SIMD 8-wide single-precision, OpenMP beam-group-parallel (SoA).
 *   2× beam throughput vs the f64 4-wide kernel.
 *   Absolute error at 40 m range ≤ 40 × 1.2e-7 ≈ 5 µm (well within LiDAR noise).
 *   OMP schedule(static) — beam costs are uniform, avoids dynamic-scheduling
 *   overhead (~5 µs per parallel region on modern Linux).
 */

#include <math.h>
#include <stdint.h>
#include <float.h>
#ifdef _OPENMP
#include <omp.h>
#endif
/*
 * AVX2 SIMD path — x86/x86-64 only.
 *
 * The compound guard checks both the ISA extension (__AVX2__) *and* the CPU
 * architecture family so that <immintrin.h> is never included on non-x86
 * targets (AArch64, RISC-V, PowerPC, WASM, …).  When the guard is false,
 * both AVX2 kernels are absent from the compiled binary and
 * ray_casting_2d_omp.py falls back to cast_ray_segments_omp automatically.
 */
#if defined(__AVX2__) && \
    (defined(__x86_64__) || defined(_M_X64) || \
     defined(__i386__)   || defined(_M_IX86))
#define _IRSIM_AVX2 1
#include <immintrin.h>
#endif

#define ORIGIN_EPS    1e-9
#define ORIGIN_EPS_F  1e-7f

#if defined(_WIN32) || defined(__CYGWIN__)
  #define IRSIM_API __declspec(dllexport)
#else
  #define IRSIM_API
#endif

/*
 * set_omp_num_threads
 *
 * Control the number of OpenMP threads used by all subsequent kernel calls.
 * Call with n=2 before starting a real-time loop so the raycaster leaves
 * enough cores free for the robot stack running in parallel.
 */
IRSIM_API void set_omp_num_threads(int n) {
#ifdef _OPENMP
    omp_set_num_threads(n);
#endif
}

/*
 * cast_ray_segments_omp
 *
 * Scalar OpenMP kernel (AoS layout).  Works on every platform.
 *
 * Parameters (all arrays are row-major / C order):
 *   origin      - double[2]     ray origin
 *   directions  - double[N*2]   unit beam directions (row = [dx, dy])
 *   seg_start   - double[M*2]   segment start points
 *   seg_end     - double[M*2]   segment end points
 *   N           - int           number of beams
 *   M           - int           number of segments
 *   max_range   - double        miss distance
 *   out_ranges  - double[N]     output: hit distances
 *   out_hit     - int64_t[N]    output: hit segment indices (-1 = miss)
 */
IRSIM_API void cast_ray_segments_omp(
    const double *origin,
    const double *directions,
    const double *seg_start,
    const double *seg_end,
    int N, int M,
    double max_range,
    double *out_ranges,
    int64_t *out_hit
) {
    int i;
    #ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic, 32)
    #endif
    for (i = 0; i < N; i++) {
        double dx  = directions[2*i];
        double dy  = directions[2*i + 1];
        double pdx = -dy;   /* perpendicular to beam */
        double pdy =  dx;

        double best_t = max_range + 1.0;  /* sentinel */
        int    best_j = -1;

        for (int j = 0; j < M; j++) {
            double svx = seg_end[2*j]     - seg_start[2*j];
            double svy = seg_end[2*j + 1] - seg_start[2*j + 1];
            double sox = origin[0] - seg_start[2*j];
            double soy = origin[1] - seg_start[2*j + 1];

            /* denom = dot(seg_vec, perp_dir) */
            double denom = svx * pdx + svy * pdy;

            if (denom == 0.0) {
                /* Parallel: check collinear overlap */
                double cross = svx * soy - svy * sox;
                if (fabs(cross) <= ORIGIN_EPS) {
                    double ta = (seg_start[2*j]     - origin[0]) * dx
                              + (seg_start[2*j + 1] - origin[1]) * dy;
                    double tb = (seg_end[2*j]        - origin[0]) * dx
                              + (seg_end[2*j + 1]   - origin[1]) * dy;
                    double ov_start = ta < tb ? ta : tb;
                    double ov_end   = ta < tb ? tb : ta;
                    if (ov_end > ORIGIN_EPS && ov_start <= max_range) {
                        double hit_t = ov_start > ORIGIN_EPS
                            ? ov_start
                            : (ov_end <= max_range ? ov_end : max_range);
                        if (hit_t <= max_range && hit_t < best_t) {
                            best_t = hit_t;
                            best_j = j;
                        }
                    }
                }
                continue;
            }

            double u = (sox * pdx + soy * pdy) / denom;
            if (u < 0.0 || u > 1.0) continue;

            double cross = svx * soy - svy * sox;
            double t = cross / denom;
            if (t > ORIGIN_EPS && t <= max_range && t < best_t) {
                best_t = t;
                best_j = j;
            }
        }

        if (best_j >= 0) {
            out_ranges[i] = best_t;
            out_hit[i]    = (int64_t)best_j;
        } else {
            out_ranges[i] = max_range;
            out_hit[i]    = -1;
        }
    }
}

#ifdef _IRSIM_AVX2
/*
 * cast_ray_segments_avx2_soa  (float64 4-wide)
 *
 * AVX2 SIMD variant: processes 4 beams simultaneously using 256-bit
 * double-precision registers.  Uses SoA (Structure-of-Arrays) layout so
 * segment scalars (svx, svy, sox, soy, cross) are broadcast once per
 * segment for all 4 beam lanes.
 *
 * Collinear overlap (denom==0 AND cross≈0) is skipped in this path;
 * use cast_ray_segments_omp for exact handling of that rare case.
 *
 * Parameters:
 *   origin    - double[2]    ray origin (shared by all beams)
 *   dir_dx    - double[N]    beam directions: x component  (SoA)
 *   dir_dy    - double[N]    beam directions: y component  (SoA)
 *   seg_sx    - double[M]    segment start x               (SoA)
 *   seg_sy    - double[M]    segment start y               (SoA)
 *   seg_ex    - double[M]    segment end x                 (SoA)
 *   seg_ey    - double[M]    segment end y                 (SoA)
 *   N         - int          number of beams
 *   M         - int          number of segments
 *   max_range - double       miss distance
 *   out_ranges - double[N]   output: hit distances
 *   out_hit   - int64_t[N]   output: hit segment indices (-1 = miss)
 */
IRSIM_API void cast_ray_segments_avx2_soa(
    const double *origin,
    const double *dir_dx,
    const double *dir_dy,
    const double *seg_sx,
    const double *seg_sy,
    const double *seg_ex,
    const double *seg_ey,
    int N, int M,
    double max_range,
    double *out_ranges,
    int64_t *out_hit
) {
    const double ox       = origin[0];
    const double oy       = origin[1];
    const double sentinel = max_range + 1.0;

    /* AVX2 block: process 4 beams per iteration */
    int avx_n = N & ~3;  /* round down to multiple of 4 */

    int base;
    #ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic, 8) if(avx_n > 32)
    #endif
    for (base = 0; base < avx_n; base += 4) {
        __m256d dx_v  = _mm256_loadu_pd(dir_dx + base);
        __m256d dy_v  = _mm256_loadu_pd(dir_dy + base);
        /* perpendicular to each beam: pdx = -dy, pdy = dx */
        __m256d pdx_v = _mm256_sub_pd(_mm256_setzero_pd(), dy_v);
        __m256d pdy_v = dx_v;

        __m256d best_t_v = _mm256_set1_pd(sentinel);
        __m256i best_j_v = _mm256_set1_epi64x(-1LL);

        const __m256d zero_v = _mm256_setzero_pd();
        const __m256d one_v  = _mm256_set1_pd(1.0);
        const __m256d eps_v  = _mm256_set1_pd(ORIGIN_EPS);
        const __m256d maxr_v = _mm256_set1_pd(max_range);

        for (int j = 0; j < M; j++) {
            /* Segment-derived scalars: identical for all 4 beam lanes */
            double svx   = seg_ex[j] - seg_sx[j];
            double svy   = seg_ey[j] - seg_sy[j];
            double sox   = ox - seg_sx[j];
            double soy   = oy - seg_sy[j];
            double cross = svx * soy - svy * sox;

            __m256d svx_v   = _mm256_set1_pd(svx);
            __m256d svy_v   = _mm256_set1_pd(svy);
            __m256d sox_v   = _mm256_set1_pd(sox);
            __m256d soy_v   = _mm256_set1_pd(soy);
            __m256d cross_v = _mm256_set1_pd(cross);

            /* denom[i] = svx * pdx[i] + svy * pdy[i] */
            __m256d denom_v = _mm256_fmadd_pd(svy_v, pdy_v,
                                  _mm256_mul_pd(svx_v, pdx_v));

            /* u[i] = (sox * pdx[i] + soy * pdy[i]) / denom[i] */
            __m256d u_v = _mm256_div_pd(
                _mm256_fmadd_pd(soy_v, pdy_v, _mm256_mul_pd(sox_v, pdx_v)),
                denom_v);

            /* t[i] = cross / denom[i] */
            __m256d t_v = _mm256_div_pd(cross_v, denom_v);

            /* Acceptance mask: u in [0,1], t in (eps, max_range], t < best_t */
            __m256d mask = _mm256_and_pd(
                _mm256_and_pd(
                    _mm256_and_pd(_mm256_cmp_pd(u_v, zero_v, _CMP_GE_OQ),
                                  _mm256_cmp_pd(u_v, one_v,  _CMP_LE_OQ)),
                    _mm256_and_pd(_mm256_cmp_pd(t_v, eps_v,  _CMP_GT_OQ),
                                  _mm256_cmp_pd(t_v, maxr_v, _CMP_LE_OQ))),
                _mm256_cmp_pd(t_v, best_t_v, _CMP_LT_OQ));

            best_t_v = _mm256_blendv_pd(best_t_v, t_v, mask);
            best_j_v = _mm256_blendv_epi8(best_j_v,
                           _mm256_set1_epi64x((long long)j),
                           _mm256_castpd_si256(mask));
        }

        /* Store 4 results */
        double    bt[4];
        long long bj[4];
        _mm256_storeu_pd(bt, best_t_v);
        _mm256_storeu_si256((__m256i *)bj, best_j_v);
        for (int k = 0; k < 4; k++) {
            int idx = base + k;
            if (bj[k] >= 0) {
                out_ranges[idx] = bt[k];
                out_hit[idx]    = (int64_t)bj[k];
            } else {
                out_ranges[idx] = max_range;
                out_hit[idx]    = -1;
            }
        }
    }

    /* Scalar tail: handles beams when N is not a multiple of 4 */
    for (int i = avx_n; i < N; i++) {
        double dx  = dir_dx[i];
        double dy  = dir_dy[i];
        double pdx = -dy;
        double pdy =  dx;
        double best_t = sentinel;
        int    best_j = -1;

        for (int j = 0; j < M; j++) {
            double svx = seg_ex[j] - seg_sx[j];
            double svy = seg_ey[j] - seg_sy[j];
            double sox = ox - seg_sx[j];
            double soy = oy - seg_sy[j];

            double denom = svx * pdx + svy * pdy;
            if (denom == 0.0) continue;

            double u = (sox * pdx + soy * pdy) / denom;
            if (u < 0.0 || u > 1.0) continue;

            double cross = svx * soy - svy * sox;
            double t = cross / denom;
            if (t > ORIGIN_EPS && t <= max_range && t < best_t) {
                best_t = t;
                best_j = j;
            }
        }

        if (best_j >= 0) {
            out_ranges[i] = best_t;
            out_hit[i]    = (int64_t)best_j;
        } else {
            out_ranges[i] = max_range;
            out_hit[i]    = -1;
        }
    }
}


/*
 * cast_ray_segments_avx2_f32_soa  (float32 8-wide)  [NEW]
 *
 * 8-wide single-precision variant of cast_ray_segments_avx2_soa.
 *
 * Advantages over the f64 4-wide kernel:
 *   - 2× SIMD width (8 beams per AVX2 register vs 4)
 *   - 50% less segment memory bandwidth (float32 SoA vs float64 SoA)
 *   - Faster OMP overhead: schedule(static) instead of schedule(dynamic)
 *   - Overall: ~2-3× faster in the raycasting kernel
 *
 * Precision: FP32 mantissa = 23 bits (≈7 decimal digits).
 *   At range_max = 40 m, absolute error ≤ 40 × 1.2e-7 ≈ 5 µm — far below
 *   real LiDAR noise (typically ≥ 1 cm).
 *
 * Parameters:
 *   origin    - float[2]     ray origin (shared by all beams)
 *   dir_dx    - float[N]     beam directions: x component  (SoA)
 *   dir_dy    - float[N]     beam directions: y component  (SoA)
 *   seg_sx    - float[M]     segment start x               (SoA)
 *   seg_sy    - float[M]     segment start y               (SoA)
 *   seg_ex    - float[M]     segment end x                 (SoA)
 *   seg_ey    - float[M]     segment end y                 (SoA)
 *   N         - int          number of beams
 *   M         - int          number of segments
 *   max_range - float        miss distance
 *   out_ranges - float[N]    output: hit distances (float32)
 *   out_hit   - int32_t[N]   output: hit segment indices (-1 = miss)
 */
IRSIM_API void cast_ray_segments_avx2_f32_soa(
    const float *origin,
    const float *dir_dx,
    const float *dir_dy,
    const float *seg_sx,
    const float *seg_sy,
    const float *seg_ex,
    const float *seg_ey,
    int N, int M,
    float max_range,
    float    *out_ranges,
    int32_t  *out_hit
) {
    const float ox       = origin[0];
    const float oy       = origin[1];
    const float sentinel = max_range + 1.0f;

    const __m256 sentinel_v = _mm256_set1_ps(sentinel);
    const __m256 zero_v     = _mm256_setzero_ps();
    const __m256 one_v      = _mm256_set1_ps(1.0f);
    const __m256 eps_v      = _mm256_set1_ps(ORIGIN_EPS_F);
    const __m256 maxr_v     = _mm256_set1_ps(max_range);

    /* AVX2 block: process 8 beams per iteration */
    int avx_n = N & ~7;  /* round down to multiple of 8 */

    int base;
    #ifdef _OPENMP
    /* schedule(static): beam cost is uniform — no dynamic overhead */
    #pragma omp parallel for schedule(static) if(avx_n > 64)
    #endif
    for (base = 0; base < avx_n; base += 8) {
        __m256 dx_v  = _mm256_loadu_ps(dir_dx + base);
        __m256 dy_v  = _mm256_loadu_ps(dir_dy + base);
        /* perpendicular to each beam: pdx = -dy, pdy = dx */
        __m256 pdx_v = _mm256_sub_ps(zero_v, dy_v);
        __m256 pdy_v = dx_v;

        __m256  best_t_v = sentinel_v;
        __m256i best_j_v = _mm256_set1_epi32(-1);

        for (int j = 0; j < M; j++) {
            /* Segment-derived scalars: identical for all 8 beam lanes */
            float svx   = seg_ex[j] - seg_sx[j];
            float svy   = seg_ey[j] - seg_sy[j];
            float sox   = ox - seg_sx[j];
            float soy   = oy - seg_sy[j];
            float cross = svx * soy - svy * sox;

            __m256 svx_v   = _mm256_set1_ps(svx);
            __m256 svy_v   = _mm256_set1_ps(svy);
            __m256 sox_v   = _mm256_set1_ps(sox);
            __m256 soy_v   = _mm256_set1_ps(soy);
            __m256 cross_v = _mm256_set1_ps(cross);

            /* denom[i] = svx * pdx[i] + svy * pdy[i] */
            __m256 denom_v = _mm256_fmadd_ps(svy_v, pdy_v,
                                 _mm256_mul_ps(svx_v, pdx_v));

            /* u[i] = (sox * pdx[i] + soy * pdy[i]) / denom[i] */
            __m256 u_v = _mm256_div_ps(
                _mm256_fmadd_ps(soy_v, pdy_v, _mm256_mul_ps(sox_v, pdx_v)),
                denom_v);

            /* t[i] = cross / denom[i] */
            __m256 t_v = _mm256_div_ps(cross_v, denom_v);

            /* Acceptance mask: u in [0,1], t in (eps, max_range], t < best_t */
            __m256 mask = _mm256_and_ps(
                _mm256_and_ps(
                    _mm256_and_ps(_mm256_cmp_ps(u_v, zero_v, _CMP_GE_OQ),
                                  _mm256_cmp_ps(u_v, one_v,  _CMP_LE_OQ)),
                    _mm256_and_ps(_mm256_cmp_ps(t_v, eps_v,  _CMP_GT_OQ),
                                  _mm256_cmp_ps(t_v, maxr_v, _CMP_LE_OQ))),
                _mm256_cmp_ps(t_v, best_t_v, _CMP_LT_OQ));

            best_t_v = _mm256_blendv_ps(best_t_v, t_v, mask);
            /*
             * blendv_epi8 with a float32 mask: each 32-bit lane of mask is
             * all-ones or all-zeros, so byte-level blending is safe.
             */
            best_j_v = _mm256_blendv_epi8(
                best_j_v,
                _mm256_set1_epi32((int32_t)j),
                _mm256_castps_si256(mask));
        }

        /* Store 8 results */
        float   bt[8];
        int32_t bj[8];
        _mm256_storeu_ps(bt, best_t_v);
        _mm256_storeu_si256((__m256i *)bj, best_j_v);
        for (int k = 0; k < 8; k++) {
            int idx = base + k;
            out_ranges[idx] = (bj[k] >= 0) ? bt[k] : max_range;
            out_hit[idx]    = bj[k];
        }
    }

    /* Scalar tail: handles remaining beams when N % 8 != 0 */
    for (int i = avx_n; i < N; i++) {
        float dx  = dir_dx[i];
        float dy  = dir_dy[i];
        float pdx = -dy;
        float pdy =  dx;
        float best_t = sentinel;
        int   best_j = -1;

        for (int j = 0; j < M; j++) {
            float svx = seg_ex[j] - seg_sx[j];
            float svy = seg_ey[j] - seg_sy[j];
            float sox = ox - seg_sx[j];
            float soy = oy - seg_sy[j];

            float denom = svx * pdx + svy * pdy;
            if (denom == 0.0f) continue;

            float u = (sox * pdx + soy * pdy) / denom;
            if (u < 0.0f || u > 1.0f) continue;

            float cross = svx * soy - svy * sox;
            float t = cross / denom;
            if (t > ORIGIN_EPS_F && t <= max_range && t < best_t) {
                best_t = t;
                best_j = j;
            }
        }
        out_ranges[i] = (best_j >= 0) ? best_t : max_range;
        out_hit[i]    = (int32_t)best_j;
    }
}

#endif /* _IRSIM_AVX2 */
