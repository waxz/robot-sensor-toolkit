/**
 * lidar_embree.cpp  —  Embree4 BVH ray caster for LiDAR 2D and 3D (pybind11)
 *
 * Two classes exported to Python:
 *
 *   EmbreeScene2D  — builds BVH from 2D segments (extruded to thin 3D quads).
 *                    cast_inplace() is a drop-in for cast_ray_segments_avx2_f32_inplace.
 *
 *   EmbreeScene3D  — builds BVH from a 3D triangle mesh.
 *                    cast_3d_lidar() matches Scene3D.cast_3d_lidar().
 *
 * Platform requirements:
 *   Ubuntu x86_64 : apt install libembree-dev  (Ubuntu 20.04+ ships Embree 4)
 *   Windows x86_64: https://github.com/RenderKit/embree/releases (pre-built DLL)
 *   macOS x86/ARM : brew install embree
 *
 * Build (Linux, fast path):
 *   g++ -O3 -march=native -ffast-math -shared -fPIC \
 *       $(python3 -m pybind11 --includes) \
 *       -I/usr/include/embree4 \
 *       lidar_embree.cpp \
 *       -lembree4 \
 *       -o lidar_embree$(python3-config --extension-suffix)
 *
 * See CMakeLists.txt for the CMake/scikit-build-core approach.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <embree4/rtcore.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

#ifdef _OPENMP
#  include <omp.h>
#endif

namespace py = pybind11;

// ── Compile-time constants ────────────────────────────────────────────────────
static constexpr float EXTRUDE_H = 1.0f;       // 2D→3D extrusion height (m)
static constexpr float RAY_Z     = 0.5f;        // ray origin z (middle of extrusion)
static constexpr float TNEAR     = 1e-5f;

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline float deg2rad(float d) { return d * (static_cast<float>(M_PI) / 180.f); }

// Raise a Python RuntimeError for a failed rtcGetDeviceError.
static void _check_device(RTCDevice dev, const char* context) {
    RTCError err = rtcGetDeviceError(dev);
    if (err != RTC_ERROR_NONE) {
        throw std::runtime_error(std::string(context) + ": rtc error " + std::to_string(err));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// EmbreeScene2D — 2D ray-segment caster via extruded 3D quads
// ═══════════════════════════════════════════════════════════════════════════════
struct EmbreeScene2D {
    RTCDevice device_ = nullptr;
    RTCScene  scene_  = nullptr;
    int       n_segs_ = 0;

    EmbreeScene2D()  = default;
    ~EmbreeScene2D() { _release(); }

    // Not copyable
    EmbreeScene2D(const EmbreeScene2D&)            = delete;
    EmbreeScene2D& operator=(const EmbreeScene2D&) = delete;

    // ── build ──────────────────────────────────────────────────────────────────
    /**
     * Build BVH from 2D segment array.
     *
     * segs_arr: float32 ndarray [N, 4] — columns: ax, ay, bx, by
     *
     * Each segment is extruded to a thin vertical quad (z: 0 → EXTRUDE_H) and
     * split into two triangles.  Rays are cast horizontally at z = RAY_Z.
     */
    void build(py::array_t<float, py::array::c_style> segs_arr) {
        auto b = segs_arr.unchecked<2>();
        if (b.shape(1) != 4)
            throw std::invalid_argument("segs must be shape [N, 4]: ax ay bx by");
        n_segs_ = (int)b.shape(0);

        _release();
        device_ = rtcNewDevice(nullptr);
        if (!device_) throw std::runtime_error("rtcNewDevice failed");
        scene_  = rtcNewScene(device_);
        if (!scene_) { rtcReleaseDevice(device_); device_=nullptr; throw std::runtime_error("rtcNewScene failed"); }

        // 4 vertices per segment, 2 triangles per segment
        RTCGeometry geom = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);
        auto* verts = static_cast<float*>(
            rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0,
                                    RTC_FORMAT_FLOAT3, 3*sizeof(float),
                                    4 * n_segs_));
        auto* tris = static_cast<unsigned*>(
            rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0,
                                    RTC_FORMAT_UINT3, 3*sizeof(unsigned),
                                    2 * n_segs_));

        for (int i = 0; i < n_segs_; ++i) {
            float ax = b(i,0), ay = b(i,1), bx = b(i,2), by = b(i,3);
            // Quad vertices: v0=(ax,ay,0) v1=(bx,by,0) v2=(bx,by,H) v3=(ax,ay,H)
            float* v = verts + i * 12;
            v[0]=ax; v[1]=ay; v[2]=0.f;
            v[3]=bx; v[4]=by; v[5]=0.f;
            v[6]=bx; v[7]=by; v[8]=EXTRUDE_H;
            v[9]=ax; v[10]=ay; v[11]=EXTRUDE_H;
            // Two triangles per quad
            unsigned* t = tris + i * 6;
            unsigned base = (unsigned)(i * 4);
            t[0]=base; t[1]=base+1; t[2]=base+2;
            t[3]=base; t[4]=base+2; t[5]=base+3;
        }

        rtcCommitGeometry(geom);
        rtcAttachGeometry(scene_, geom);
        rtcReleaseGeometry(geom);
        rtcCommitScene(scene_);
        _check_device(device_, "build");
    }

    // ── cast_inplace ─────────────────────────────────────────────────────────
    /**
     * Drop-in for cast_ray_segments_avx2_f32_inplace.
     *
     * origin_f   : float32 ndarray [2]  — (ox, oy) in world frame
     * dir_dx_f   : float32 ndarray [N]  — x components of beam directions (unit)
     * dir_dy_f   : float32 ndarray [N]  — y components of beam directions (unit)
     * max_range_f: float                — maximum range (m)
     * out_ranges_f: float32 ndarray [N] — written in-place (range per beam)
     * out_hit_i  : int32 ndarray  [N]   — written in-place (triangle index / 2
     *                                     gives segment index; -1 on miss)
     *
     * Note: Unlike the AVX2 kernel the scene segments are pre-loaded at build()
     * time.  The seg_sx/sy/ex/ey parameters are accepted but ignored (kept for
     * API compatibility with the existing Lidar2D._step_fast path).
     */
    void cast_inplace(
        py::array_t<float, py::array::c_style> origin_f,
        py::array_t<float, py::array::c_style> dir_dx_f,
        py::array_t<float, py::array::c_style> dir_dy_f,
        float max_range_f,
        py::array_t<float, py::array::c_style> out_ranges_f,
        py::array_t<int,   py::array::c_style> out_hit_i
    ) {
        if (!scene_) throw std::runtime_error("EmbreeScene2D: scene not built");

        auto ox = origin_f.unchecked<1>();
        auto dx = dir_dx_f.unchecked<1>();
        auto dy = dir_dy_f.unchecked<1>();
        auto  r = out_ranges_f.mutable_unchecked<1>();
        auto  h = out_hit_i.mutable_unchecked<1>();

        const int N = (int)dx.shape(0);
        if ((int)r.shape(0) != N || (int)h.shape(0) != N)
            throw std::invalid_argument("output arrays must have same length as dir_dx_f");

        const float ox_val = ox(0), oy_val = ox(1);  // origin_f[2]

        struct RTCRayHit rh;

        for (int i = 0; i < N; ++i) {
            // Zero-init hit
            rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            rh.hit.primID = RTC_INVALID_GEOMETRY_ID;
            rh.ray.org_x  = ox_val;
            rh.ray.org_y  = oy_val;
            rh.ray.org_z  = RAY_Z;
            rh.ray.dir_x  = dx(i);
            rh.ray.dir_y  = dy(i);
            rh.ray.dir_z  = 0.f;
            rh.ray.tnear  = TNEAR;
            rh.ray.tfar   = max_range_f;
            rh.ray.mask   = 0xFFFFFFFF;
            rh.ray.flags  = 0;
            rh.hit.Ng_x = rh.hit.Ng_y = rh.hit.Ng_z = 0.f;
            rh.hit.u = rh.hit.v = 0.f;
            rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

            struct RTCIntersectArguments iargs;
            rtcInitIntersectArguments(&iargs);
            rtcIntersect1(scene_, &rh, &iargs);

            if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
                r(i) = rh.ray.tfar;
                // primID is the triangle index; segment = primID / 2
                h(i) = (int)(rh.hit.primID / 2);
            } else {
                r(i) = max_range_f;
                h(i) = -1;
            }
        }
    }

    // ── cast8 ─────────────────────────────────────────────────────────────────
    /**
     * 8-ray SIMD packet cast (AVX2 path, ~1.5-2× faster on long scans).
     * Same semantics as cast_inplace but processes 8 rays per Embree call.
     */
    void cast8_inplace(
        py::array_t<float, py::array::c_style> origin_f,
        py::array_t<float, py::array::c_style> dir_dx_f,
        py::array_t<float, py::array::c_style> dir_dy_f,
        float max_range_f,
        py::array_t<float, py::array::c_style> out_ranges_f,
        py::array_t<int,   py::array::c_style> out_hit_i
    ) {
        if (!scene_) throw std::runtime_error("EmbreeScene2D: scene not built");

        auto ox = origin_f.unchecked<1>();
        auto dx = dir_dx_f.unchecked<1>();
        auto dy = dir_dy_f.unchecked<1>();
        auto  r = out_ranges_f.mutable_unchecked<1>();
        auto  h = out_hit_i.mutable_unchecked<1>();

        const int N = (int)dx.shape(0);
        const float ox_val = ox(0), oy_val = ox(1);

        // Process in batches of 8; scalar fallback for the remainder
        int i = 0;
        for (; i + 8 <= N; i += 8) {
            struct RTCRayHit8 rh8;
            int valid[8];
            for (int k = 0; k < 8; ++k) {
                valid[k] = -1;
                rh8.ray.org_x[k]  = ox_val;
                rh8.ray.org_y[k]  = oy_val;
                rh8.ray.org_z[k]  = RAY_Z;
                rh8.ray.dir_x[k]  = dx(i+k);
                rh8.ray.dir_y[k]  = dy(i+k);
                rh8.ray.dir_z[k]  = 0.f;
                rh8.ray.tnear[k]  = TNEAR;
                rh8.ray.tfar[k]   = max_range_f;
                rh8.ray.mask[k]   = 0xFFFFFFFF;
                rh8.ray.flags[k]  = 0;
                rh8.ray.time[k]   = 0.f;
                rh8.hit.geomID[k] = RTC_INVALID_GEOMETRY_ID;
                rh8.hit.primID[k] = RTC_INVALID_GEOMETRY_ID;
                rh8.hit.instID[0][k] = RTC_INVALID_GEOMETRY_ID;
            }
            struct RTCIntersectArguments iargs;
            rtcInitIntersectArguments(&iargs);
            rtcIntersect8(valid, scene_, &rh8, &iargs);
            for (int k = 0; k < 8; ++k) {
                if (rh8.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID) {
                    r(i+k) = rh8.ray.tfar[k];
                    h(i+k) = (int)(rh8.hit.primID[k] / 2);
                } else {
                    r(i+k) = max_range_f;
                    h(i+k) = -1;
                }
            }
        }
        // Scalar remainder
        for (; i < N; ++i) {
            struct RTCRayHit rh;
            rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            rh.hit.primID = RTC_INVALID_GEOMETRY_ID;
            rh.ray.org_x  = ox_val; rh.ray.org_y = oy_val; rh.ray.org_z = RAY_Z;
            rh.ray.dir_x  = dx(i); rh.ray.dir_y = dy(i); rh.ray.dir_z = 0.f;
            rh.ray.tnear  = TNEAR; rh.ray.tfar   = max_range_f;
            rh.ray.mask   = 0xFFFFFFFF; rh.ray.flags = 0;
            rh.hit.Ng_x = rh.hit.Ng_y = rh.hit.Ng_z = 0.f;
            rh.hit.u = rh.hit.v = 0.f;
            rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            struct RTCIntersectArguments iargs;
            rtcInitIntersectArguments(&iargs);
            rtcIntersect1(scene_, &rh, &iargs);
            if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
                r(i) = rh.ray.tfar;
                h(i) = (int)(rh.hit.primID / 2);
            } else {
                r(i) = max_range_f;
                h(i) = -1;
            }
        }
    }

    // ── cast16 ────────────────────────────────────────────────────────────────
    /**
     * 16-ray AVX-512 packet cast.  Processes 16 rays per rtcIntersect16 call,
     * falls back to packet8 then scalar for remainders.
     */
    void cast16_inplace(
        py::array_t<float, py::array::c_style> origin_f,
        py::array_t<float, py::array::c_style> dir_dx_f,
        py::array_t<float, py::array::c_style> dir_dy_f,
        float max_range_f,
        py::array_t<float, py::array::c_style> out_ranges_f,
        py::array_t<int,   py::array::c_style> out_hit_i
    ) {
        if (!scene_) throw std::runtime_error("EmbreeScene2D: scene not built");

        auto ox = origin_f.unchecked<1>();
        auto dx = dir_dx_f.unchecked<1>();
        auto dy = dir_dy_f.unchecked<1>();
        auto  r = out_ranges_f.mutable_unchecked<1>();
        auto  h = out_hit_i.mutable_unchecked<1>();

        const int N = (int)dx.shape(0);
        const float ox_val = ox(0), oy_val = ox(1);

        int i = 0;
        // ── packet16 (AVX-512) blocks ────────────────────────────────────────
        for (; i + 16 <= N; i += 16) {
            struct RTCRayHit16 rh16;
            int valid[16];
            for (int k = 0; k < 16; ++k) {
                valid[k] = -1;
                rh16.ray.org_x[k]  = ox_val;
                rh16.ray.org_y[k]  = oy_val;
                rh16.ray.org_z[k]  = RAY_Z;
                rh16.ray.dir_x[k]  = dx(i+k);
                rh16.ray.dir_y[k]  = dy(i+k);
                rh16.ray.dir_z[k]  = 0.f;
                rh16.ray.tnear[k]  = TNEAR;
                rh16.ray.tfar[k]   = max_range_f;
                rh16.ray.mask[k]   = 0xFFFFFFFF;
                rh16.ray.flags[k]  = 0;
                rh16.ray.time[k]   = 0.f;
                rh16.hit.geomID[k] = RTC_INVALID_GEOMETRY_ID;
                rh16.hit.primID[k] = RTC_INVALID_GEOMETRY_ID;
                rh16.hit.instID[0][k] = RTC_INVALID_GEOMETRY_ID;
            }
            struct RTCIntersectArguments iargs;
            rtcInitIntersectArguments(&iargs);
            rtcIntersect16(valid, scene_, &rh16, &iargs);
            for (int k = 0; k < 16; ++k) {
                if (rh16.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID) {
                    r(i+k) = rh16.ray.tfar[k];
                    h(i+k) = (int)(rh16.hit.primID[k] / 2);
                } else {
                    r(i+k) = max_range_f;
                    h(i+k) = -1;
                }
            }
        }
        // ── packet8 remainder ────────────────────────────────────────────────
        for (; i + 8 <= N; i += 8) {
            struct RTCRayHit8 rh8;
            int valid[8];
            for (int k = 0; k < 8; ++k) {
                valid[k] = -1;
                rh8.ray.org_x[k]  = ox_val;
                rh8.ray.org_y[k]  = oy_val;
                rh8.ray.org_z[k]  = RAY_Z;
                rh8.ray.dir_x[k]  = dx(i+k);
                rh8.ray.dir_y[k]  = dy(i+k);
                rh8.ray.dir_z[k]  = 0.f;
                rh8.ray.tnear[k]  = TNEAR;
                rh8.ray.tfar[k]   = max_range_f;
                rh8.ray.mask[k]   = 0xFFFFFFFF;
                rh8.ray.flags[k]  = 0;
                rh8.ray.time[k]   = 0.f;
                rh8.hit.geomID[k] = RTC_INVALID_GEOMETRY_ID;
                rh8.hit.primID[k] = RTC_INVALID_GEOMETRY_ID;
                rh8.hit.instID[0][k] = RTC_INVALID_GEOMETRY_ID;
            }
            struct RTCIntersectArguments iargs;
            rtcInitIntersectArguments(&iargs);
            rtcIntersect8(valid, scene_, &rh8, &iargs);
            for (int k = 0; k < 8; ++k) {
                if (rh8.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID) {
                    r(i+k) = rh8.ray.tfar[k];
                    h(i+k) = (int)(rh8.hit.primID[k] / 2);
                } else {
                    r(i+k) = max_range_f;
                    h(i+k) = -1;
                }
            }
        }
        // ── scalar tail ──────────────────────────────────────────────────────
        for (; i < N; ++i) {
            struct RTCRayHit rh;
            rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            rh.hit.primID = RTC_INVALID_GEOMETRY_ID;
            rh.ray.org_x  = ox_val; rh.ray.org_y = oy_val; rh.ray.org_z = RAY_Z;
            rh.ray.dir_x  = dx(i); rh.ray.dir_y = dy(i); rh.ray.dir_z = 0.f;
            rh.ray.tnear  = TNEAR; rh.ray.tfar   = max_range_f;
            rh.ray.mask   = 0xFFFFFFFF; rh.ray.flags = 0;
            rh.hit.Ng_x = rh.hit.Ng_y = rh.hit.Ng_z = 0.f;
            rh.hit.u = rh.hit.v = 0.f;
            rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            struct RTCIntersectArguments iargs;
            rtcInitIntersectArguments(&iargs);
            rtcIntersect1(scene_, &rh, &iargs);
            if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
                r(i) = rh.ray.tfar;
                h(i) = (int)(rh.hit.primID / 2);
            } else {
                r(i) = max_range_f;
                h(i) = -1;
            }
        }
    }

    int n_segments() const { return n_segs_; }

private:
    void _release() {
        if (scene_)  { rtcReleaseScene(scene_);   scene_  = nullptr; }
        if (device_) { rtcReleaseDevice(device_); device_ = nullptr; }
    }
};


// ═══════════════════════════════════════════════════════════════════════════════
// EmbreeScene3D — 3D triangle mesh BVH for spinning LiDAR simulation
// ═══════════════════════════════════════════════════════════════════════════════
struct EmbreeScene3D {
    RTCDevice device_ = nullptr;
    RTCScene  scene_  = nullptr;

    EmbreeScene3D()  = default;
    ~EmbreeScene3D() { _release(); }

    EmbreeScene3D(const EmbreeScene3D&)            = delete;
    EmbreeScene3D& operator=(const EmbreeScene3D&) = delete;

    // ── build ──────────────────────────────────────────────────────────────────
    /**
     * Build Embree BVH from a triangle mesh.
     *
     * vertices : float32 [V, 3]   — x, y, z per vertex
     * triangles: int32  [T, 3]    — vertex indices per triangle
     */
    void build(
        py::array_t<float, py::array::c_style> vertices,
        py::array_t<int,   py::array::c_style> triangles
    ) {
        auto vb = vertices.unchecked<2>();
        auto tb = triangles.unchecked<2>();
        if (vb.shape(1) != 3) throw std::invalid_argument("vertices must be [V,3]");
        if (tb.shape(1) != 3) throw std::invalid_argument("triangles must be [T,3]");
        int V = (int)vb.shape(0), T = (int)tb.shape(0);

        _release();
        device_ = rtcNewDevice(nullptr);
        if (!device_) throw std::runtime_error("rtcNewDevice failed");
        scene_  = rtcNewScene(device_);
        if (!scene_) { _release(); throw std::runtime_error("rtcNewScene failed"); }

        RTCGeometry geom = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

        auto* verts = static_cast<float*>(
            rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0,
                                    RTC_FORMAT_FLOAT3, 3*sizeof(float), V));
        for (int v = 0; v < V; ++v) {
            verts[v*3+0] = vb(v,0);
            verts[v*3+1] = vb(v,1);
            verts[v*3+2] = vb(v,2);
        }

        auto* tris = static_cast<unsigned*>(
            rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0,
                                    RTC_FORMAT_UINT3, 3*sizeof(unsigned), T));
        for (int t = 0; t < T; ++t) {
            tris[t*3+0] = (unsigned)tb(t,0);
            tris[t*3+1] = (unsigned)tb(t,1);
            tris[t*3+2] = (unsigned)tb(t,2);
        }

        rtcCommitGeometry(geom);
        rtcAttachGeometry(scene_, geom);
        rtcReleaseGeometry(geom);
        rtcCommitScene(scene_);
        _check_device(device_, "build3d");
    }

    // ── build_soup ─────────────────────────────────────────────────────────────
    /**
     * Convenience overload: accepts a triangle soup (float32 [T, 3, 3]).
     * Matches the output of build_soup() in the benchmark script.
     */
    void build_soup(py::array_t<float, py::array::c_style> soup) {
        auto b = soup.unchecked<3>();
        if (b.shape(1) != 3 || b.shape(2) != 3)
            throw std::invalid_argument("soup must be [T, 3, 3]");
        int T = (int)b.shape(0);

        _release();
        device_ = rtcNewDevice(nullptr);
        if (!device_) throw std::runtime_error("rtcNewDevice failed");
        scene_  = rtcNewScene(device_);
        if (!scene_) { _release(); throw std::runtime_error("rtcNewScene failed"); }

        RTCGeometry geom = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);
        auto* verts = static_cast<float*>(
            rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0,
                                    RTC_FORMAT_FLOAT3, 3*sizeof(float), 3*T));
        auto* tris = static_cast<unsigned*>(
            rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0,
                                    RTC_FORMAT_UINT3, 3*sizeof(unsigned), T));
        for (int t = 0; t < T; ++t) {
            for (int v = 0; v < 3; ++v) {
                verts[(t*3+v)*3+0] = b(t,v,0);
                verts[(t*3+v)*3+1] = b(t,v,1);
                verts[(t*3+v)*3+2] = b(t,v,2);
            }
            tris[t*3+0] = (unsigned)(t*3+0);
            tris[t*3+1] = (unsigned)(t*3+1);
            tris[t*3+2] = (unsigned)(t*3+2);
        }
        rtcCommitGeometry(geom);
        rtcAttachGeometry(scene_, geom);
        rtcReleaseGeometry(geom);
        rtcCommitScene(scene_);
        _check_device(device_, "build_soup");
    }

    // ── cast_3d_lidar ─────────────────────────────────────────────────────────
    /**
     * Simulate a spinning 3D LiDAR from a given origin.
     *
     * Matches the signature of Scene3D.cast_3d_lidar():
     *   origin           : list/array [3] — x, y, z of sensor
     *   n_vertical       : int             — vertical channels
     *   n_horizontal     : int             — horizontal points per revolution
     *   elev_min_deg     : float           — minimum elevation angle
     *   elev_max_deg     : float           — maximum elevation angle
     *   range_max        : float           — maximum range (m)
     *
     * Returns float32 ndarray [N_hits, 4] — columns: x, y, z, distance
     */
    py::array_t<float> cast_3d_lidar(
        py::array_t<float> origin,
        int   n_vertical,
        int   n_horizontal,
        float elev_min_deg,
        float elev_max_deg,
        float range_max
    ) {
        if (!scene_) throw std::runtime_error("EmbreeScene3D: scene not built");

        auto ob = origin.unchecked<1>();
        if (ob.shape(0) < 3) throw std::invalid_argument("origin must have 3 elements");

        const float ox = ob(0), oy = ob(1), oz = ob(2);
        const float az_step  = 2.f * (float)M_PI / (float)n_horizontal;
        const float el_step  = (n_vertical > 1)
            ? deg2rad(elev_max_deg - elev_min_deg) / (float)(n_vertical - 1)
            : 0.f;
        const float el_min   = deg2rad(elev_min_deg);

        const int N = n_vertical * n_horizontal;

        // Pre-compute all ray directions (enables OpenMP parallel cast)
        std::vector<float> dir_x(N), dir_y(N), dir_z(N);
        for (int h = 0; h < n_horizontal; ++h) {
            float az = (float)h * az_step;
            float cos_az = std::cos(az), sin_az = std::sin(az);
            for (int v = 0; v < n_vertical; ++v) {
                float el     = el_min + (float)v * el_step;
                float cos_el = std::cos(el), sin_el = std::sin(el);
                int   idx    = h * n_vertical + v;
                dir_x[idx]   = cos_el * cos_az;
                dir_y[idx]   = cos_el * sin_az;
                dir_z[idx]   = sin_el;
            }
        }

        // Per-ray results (flat: t values; invalid = -1)
        std::vector<float> t_out(N, -1.f);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
        for (int i = 0; i < N; ++i) {
            struct RTCRayHit rh;
            struct RTCIntersectArguments iargs;
            rtcInitIntersectArguments(&iargs);
            rh.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
            rh.hit.primID    = RTC_INVALID_GEOMETRY_ID;
            rh.ray.org_x     = ox; rh.ray.org_y = oy; rh.ray.org_z = oz;
            rh.ray.dir_x     = dir_x[i]; rh.ray.dir_y = dir_y[i]; rh.ray.dir_z = dir_z[i];
            rh.ray.tnear     = TNEAR; rh.ray.tfar  = range_max;
            rh.ray.mask      = 0xFFFFFFFF; rh.ray.flags = 0;
            rh.hit.Ng_x = rh.hit.Ng_y = rh.hit.Ng_z = 0.f;
            rh.hit.u = rh.hit.v = 0.f;
            rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            rtcIntersect1(scene_, &rh, &iargs);
            if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID)
                t_out[i] = rh.ray.tfar;
        }

        // Pack hits into output array
        std::vector<float> hits;
        hits.reserve(N * 4 / 2);   // rough estimate
        for (int i = 0; i < N; ++i) {
            if (t_out[i] < 0.f) continue;
            float t = t_out[i];
            hits.push_back(ox + t * dir_x[i]);
            hits.push_back(oy + t * dir_y[i]);
            hits.push_back(oz + t * dir_z[i]);
            hits.push_back(t);
        }

        int n_hits = (int)(hits.size() / 4);
        auto result = py::array_t<float>({n_hits, 4});
        std::memcpy(result.mutable_data(), hits.data(), hits.size() * sizeof(float));
        return result;
    }

    // ── cast_3d_lidar_packet16 ────────────────────────────────────────────────
    /**
     * AVX-512 packet16 + OpenMP spinning LiDAR.
     *
     * Outer loop iterates over n_horizontal azimuth steps (OMP-parallel).
     * Inner loop packs n_vertical elevation rays into packet16 blocks (ideally
     * one call for VLP-16 with n_vertical==16) then falls back to packet8 and
     * scalar for remainders.  Coherent elevation bundles share BVH subtrees,
     * giving substantially better SIMD utilisation than the scalar path.
     *
     * Same signature and return format as cast_3d_lidar().
     */
    py::array_t<float> cast_3d_lidar_packet16(
        py::array_t<float> origin,
        int   n_vertical,
        int   n_horizontal,
        float elev_min_deg,
        float elev_max_deg,
        float range_max
    ) {
        if (!scene_) throw std::runtime_error("EmbreeScene3D: scene not built");

        auto ob = origin.unchecked<1>();
        if (ob.shape(0) < 3) throw std::invalid_argument("origin must have 3 elements");

        const float ox = ob(0), oy = ob(1), oz = ob(2);
        const float az_step = 2.f * (float)M_PI / (float)n_horizontal;
        const float el_step = (n_vertical > 1)
            ? deg2rad(elev_max_deg - elev_min_deg) / (float)(n_vertical - 1)
            : 0.f;
        const float el_min = deg2rad(elev_min_deg);
        const int N = n_vertical * n_horizontal;

        // Pre-compute all ray directions (row-major: [h][v])
        std::vector<float> dir_x(N), dir_y(N), dir_z(N);
        for (int h = 0; h < n_horizontal; ++h) {
            float az = (float)h * az_step;
            float cos_az = std::cos(az), sin_az = std::sin(az);
            for (int v = 0; v < n_vertical; ++v) {
                float el = el_min + (float)v * el_step;
                float cos_el = std::cos(el), sin_el = std::sin(el);
                int idx = h * n_vertical + v;
                dir_x[idx] = cos_el * cos_az;
                dir_y[idx] = cos_el * sin_az;
                dir_z[idx] = sin_el;
            }
        }

        std::vector<float> t_out(N, -1.f);

        // OMP over azimuth steps; each step uses packet16/8/scalar for elevation
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 32)
#endif
        for (int h = 0; h < n_horizontal; ++h) {
            int v = 0;
            const int base = h * n_vertical;

            // ── packet16 blocks (AVX-512) ─────────────────────────────────
            for (; v + 16 <= n_vertical; v += 16) {
                struct RTCRayHit16 rh16;
                int valid[16];
                for (int k = 0; k < 16; ++k) {
                    int idx = base + v + k;
                    valid[k] = -1;
                    rh16.ray.org_x[k]  = ox;
                    rh16.ray.org_y[k]  = oy;
                    rh16.ray.org_z[k]  = oz;
                    rh16.ray.dir_x[k]  = dir_x[idx];
                    rh16.ray.dir_y[k]  = dir_y[idx];
                    rh16.ray.dir_z[k]  = dir_z[idx];
                    rh16.ray.tnear[k]  = TNEAR;
                    rh16.ray.tfar[k]   = range_max;
                    rh16.ray.mask[k]   = 0xFFFFFFFF;
                    rh16.ray.flags[k]  = 0;
                    rh16.ray.time[k]   = 0.f;
                    rh16.hit.geomID[k] = RTC_INVALID_GEOMETRY_ID;
                    rh16.hit.primID[k] = RTC_INVALID_GEOMETRY_ID;
                    rh16.hit.instID[0][k] = RTC_INVALID_GEOMETRY_ID;
                }
                struct RTCIntersectArguments iargs;
                rtcInitIntersectArguments(&iargs);
                rtcIntersect16(valid, scene_, &rh16, &iargs);
                for (int k = 0; k < 16; ++k) {
                    if (rh16.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID)
                        t_out[base + v + k] = rh16.ray.tfar[k];
                }
            }
            // ── packet8 remainder ─────────────────────────────────────────
            for (; v + 8 <= n_vertical; v += 8) {
                struct RTCRayHit8 rh8;
                int valid[8];
                for (int k = 0; k < 8; ++k) {
                    int idx = base + v + k;
                    valid[k] = -1;
                    rh8.ray.org_x[k]  = ox;
                    rh8.ray.org_y[k]  = oy;
                    rh8.ray.org_z[k]  = oz;
                    rh8.ray.dir_x[k]  = dir_x[idx];
                    rh8.ray.dir_y[k]  = dir_y[idx];
                    rh8.ray.dir_z[k]  = dir_z[idx];
                    rh8.ray.tnear[k]  = TNEAR;
                    rh8.ray.tfar[k]   = range_max;
                    rh8.ray.mask[k]   = 0xFFFFFFFF;
                    rh8.ray.flags[k]  = 0;
                    rh8.ray.time[k]   = 0.f;
                    rh8.hit.geomID[k] = RTC_INVALID_GEOMETRY_ID;
                    rh8.hit.primID[k] = RTC_INVALID_GEOMETRY_ID;
                    rh8.hit.instID[0][k] = RTC_INVALID_GEOMETRY_ID;
                }
                struct RTCIntersectArguments iargs;
                rtcInitIntersectArguments(&iargs);
                rtcIntersect8(valid, scene_, &rh8, &iargs);
                for (int k = 0; k < 8; ++k) {
                    if (rh8.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID)
                        t_out[base + v + k] = rh8.ray.tfar[k];
                }
            }
            // ── scalar tail ───────────────────────────────────────────────
            for (; v < n_vertical; ++v) {
                int idx = base + v;
                struct RTCRayHit rh;
                struct RTCIntersectArguments iargs;
                rtcInitIntersectArguments(&iargs);
                rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                rh.hit.primID = RTC_INVALID_GEOMETRY_ID;
                rh.ray.org_x = ox; rh.ray.org_y = oy; rh.ray.org_z = oz;
                rh.ray.dir_x = dir_x[idx]; rh.ray.dir_y = dir_y[idx]; rh.ray.dir_z = dir_z[idx];
                rh.ray.tnear = TNEAR; rh.ray.tfar = range_max;
                rh.ray.mask = 0xFFFFFFFF; rh.ray.flags = 0;
                rh.hit.Ng_x = rh.hit.Ng_y = rh.hit.Ng_z = 0.f;
                rh.hit.u = rh.hit.v = 0.f;
                rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
                rtcIntersect1(scene_, &rh, &iargs);
                if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID)
                    t_out[idx] = rh.ray.tfar;
            }
        }

        // Pack hits
        std::vector<float> hits;
        hits.reserve(N * 4 / 2);
        for (int i = 0; i < N; ++i) {
            if (t_out[i] < 0.f) continue;
            float t = t_out[i];
            hits.push_back(ox + t * dir_x[i]);
            hits.push_back(oy + t * dir_y[i]);
            hits.push_back(oz + t * dir_z[i]);
            hits.push_back(t);
        }
        int n_hits = (int)(hits.size() / 4);
        auto result = py::array_t<float>({n_hits, 4});
        std::memcpy(result.mutable_data(), hits.data(), hits.size() * sizeof(float));
        return result;
    }

    // ── cast_rays ─────────────────────────────────────────────────────────────
    /**
     * General multi-ray cast.
     *
     * origins   : float32 [N, 3]
     * directions: float32 [N, 3]
     * range_max : float
     *
     * Returns float32 [N] — range per ray (range_max on miss)
     */
    py::array_t<float> cast_rays(
        py::array_t<float, py::array::c_style> origins,
        py::array_t<float, py::array::c_style> directions,
        float range_max
    ) {
        if (!scene_) throw std::runtime_error("EmbreeScene3D: scene not built");
        auto ob = origins.unchecked<2>();
        auto db = directions.unchecked<2>();
        if (ob.shape(1) != 3 || db.shape(1) != 3)
            throw std::invalid_argument("origins and directions must be [N,3]");
        int N = (int)ob.shape(0);
        if ((int)db.shape(0) != N) throw std::invalid_argument("origins and directions must have same N");

        auto result = py::array_t<float>(N);
        auto r = result.mutable_unchecked<1>();

        struct RTCIntersectArguments iargs;
        rtcInitIntersectArguments(&iargs);

        for (int i = 0; i < N; ++i) {
            struct RTCRayHit rh;
            rh.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
            rh.hit.primID    = RTC_INVALID_GEOMETRY_ID;
            rh.ray.org_x     = ob(i,0); rh.ray.org_y = ob(i,1); rh.ray.org_z = ob(i,2);
            rh.ray.dir_x     = db(i,0); rh.ray.dir_y = db(i,1); rh.ray.dir_z = db(i,2);
            rh.ray.tnear     = TNEAR; rh.ray.tfar = range_max;
            rh.ray.mask      = 0xFFFFFFFF; rh.ray.flags = 0;
            rh.hit.Ng_x = rh.hit.Ng_y = rh.hit.Ng_z = 0.f;
            rh.hit.u = rh.hit.v = 0.f;
            rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
            rtcIntersect1(scene_, &rh, &iargs);
            r(i) = (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID) ? rh.ray.tfar : range_max;
        }
        return result;
    }

private:
    void _release() {
        if (scene_)  { rtcReleaseScene(scene_);   scene_  = nullptr; }
        if (device_) { rtcReleaseDevice(device_); device_ = nullptr; }
    }
};


// ═══════════════════════════════════════════════════════════════════════════════
// pybind11 module
// ═══════════════════════════════════════════════════════════════════════════════
PYBIND11_MODULE(lidar_embree, m) {
    m.doc() = "Embree4 BVH ray caster for LiDAR 2D and 3D simulation";

    py::class_<EmbreeScene2D>(m, "EmbreeScene2D",
        R"pbdoc(
        2D LiDAR BVH scene using Embree4.

        Segments are extruded to thin 3D vertical quads (height=1 m, cast at z=0.5 m).
        Works as a drop-in for ``cast_ray_segments_avx2_f32_inplace``.

        Usage::

            scene = lidar_embree.EmbreeScene2D()
            scene.build(segs_f32)            # [N,4] ax ay bx by
            scene.cast_inplace(origin, dx, dy, rmax, out_r, out_h)
        )pbdoc")
        .def(py::init<>())
        .def("build", &EmbreeScene2D::build,
             py::arg("segs"),
             "Build BVH from float32 [N,4] segment array (ax ay bx by).")
        .def("cast_inplace", &EmbreeScene2D::cast_inplace,
             py::arg("origin_f"), py::arg("dir_dx_f"), py::arg("dir_dy_f"),
             py::arg("max_range_f"),
             py::arg("out_ranges_f"), py::arg("out_hit_i"),
             "Scalar ray cast into pre-allocated output buffers.")
        .def("cast8_inplace", &EmbreeScene2D::cast8_inplace,
             py::arg("origin_f"), py::arg("dir_dx_f"), py::arg("dir_dy_f"),
             py::arg("max_range_f"),
             py::arg("out_ranges_f"), py::arg("out_hit_i"),
             "8-ray SIMD packet cast (AVX2 path).")
        .def("cast16_inplace", &EmbreeScene2D::cast16_inplace,
             py::arg("origin_f"), py::arg("dir_dx_f"), py::arg("dir_dy_f"),
             py::arg("max_range_f"),
             py::arg("out_ranges_f"), py::arg("out_hit_i"),
             "16-ray AVX-512 packet cast; falls back to packet8+scalar for remainders.")
        .def("n_segments", &EmbreeScene2D::n_segments,
             "Number of 2D segments in the scene.");

    py::class_<EmbreeScene3D>(m, "EmbreeScene3D",
        R"pbdoc(
        3D LiDAR BVH scene using Embree4.

        Accepts a triangle mesh and simulates a spinning multi-channel LiDAR.

        Usage::

            scene = lidar_embree.EmbreeScene3D()
            scene.build(vertices_f32, triangles_i32)
            scan = scene.cast_3d_lidar(origin, 16, 1800, -15, 15, 50.0)
            # scan: float32 [N_hits, 4] — (x, y, z, distance)
        )pbdoc")
        .def(py::init<>())
        .def("build", &EmbreeScene3D::build,
             py::arg("vertices"), py::arg("triangles"),
             "Build BVH from vertex buffer [V,3] and triangle index buffer [T,3].")
        .def("build_soup", &EmbreeScene3D::build_soup,
             py::arg("soup"),
             "Build BVH from triangle soup float32 [T,3,3].")
        .def("cast_3d_lidar", &EmbreeScene3D::cast_3d_lidar,
             py::arg("origin"),
             py::arg("n_vertical"), py::arg("n_horizontal"),
             py::arg("elev_min_deg"), py::arg("elev_max_deg"),
             py::arg("range_max"),
             "Cast a full spinning LiDAR scan. Returns float32 [N_hits, 4] (x,y,z,dist).")
        .def("cast_3d_lidar_packet16", &EmbreeScene3D::cast_3d_lidar_packet16,
             py::arg("origin"),
             py::arg("n_vertical"), py::arg("n_horizontal"),
             py::arg("elev_min_deg"), py::arg("elev_max_deg"),
             py::arg("range_max"),
             "AVX-512 packet16 + OMP spinning LiDAR scan. "
             "Optimal for VLP-16 (n_vertical=16): one rtcIntersect16 per azimuth step. "
             "Returns float32 [N_hits, 4] (x,y,z,dist).")
        .def("cast_rays", &EmbreeScene3D::cast_rays,
             py::arg("origins"), py::arg("directions"), py::arg("range_max"),
             "Cast N rays, returns float32 [N] ranges.");
}
