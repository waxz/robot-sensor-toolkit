/**
 * lidar2d_embree.cpp  —  Direct Embree4 LiDAR 2D ray caster (pybind11 wrapper)
 *
 * Replaces the full Open3D Python call chain:
 *   np.concatenate → Tensor() → cast_rays() → .numpy()   (~215 µs overhead)
 *
 * with a direct C++ path:
 *   build_scene(triangles)   — one-time BVH build
 *   cast(ox, oy, theta)      — per-step, returns numpy array of ranges
 *
 * Embree4 API: rtcIntersect1() scalar, rtcIntersect8() 8-ray SIMD packet
 * Rotation: in-place per step (same as O3DLidar2D.step())
 * Thread model: single-threaded cast (matches O3D benchmark)
 *
 * Build:
 *   g++ -O3 -march=native -ffast-math -shared -fPIC \
 *       $(python3 -m pybind11 --includes) \
 *       -I/usr/include/embree4 \
 *       lidar2d_embree.cpp \
 *       -L/usr/lib/x86_64-linux-gnu -lembree4 \
 *       -o lidar2d_embree$(python3-config --extension-suffix)
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <embree4/rtcore.h>

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace py = pybind11;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr float EXTRUDE_H = 3.0f;    // pillar height (matches _make_scene_3d)
static constexpr float RAY_ORIGIN_Z = 1.5f; // midway up the pillar
static constexpr float RAY_DIR_Z = 0.0f;    // horizontal cast
static constexpr float TNEAR = 1e-4f;
static constexpr float TFAR = 1e30f;

// ---------------------------------------------------------------------------
// Scene wrapper
// ---------------------------------------------------------------------------
struct Lidar2DEmbree {
    RTCDevice device = nullptr;
    RTCScene  scene  = nullptr;

    // Pre-allocated direction arrays (in robot frame, rotated per step)
    std::vector<float> base_dx;  // unit directions when theta=0
    std::vector<float> base_dy;
    int n_beams = 0;
    float range_max = 30.0f;

    Lidar2DEmbree() = default;

    ~Lidar2DEmbree() {
        if (scene)  rtcReleaseScene(scene);
        if (device) rtcReleaseDevice(device);
    }

    // Build BVH from triangle soup.
    // triangles: float32 array shape [N, 3, 3] — each triangle: 3 vertices × (x,y,z)
    void build_scene(py::array_t<float, py::array::c_style> triangles) {
        auto buf = triangles.unchecked<3>();
        if (buf.shape(1) != 3 || buf.shape(2) != 3)
            throw std::runtime_error("triangles must be shape [N,3,3]");
        int n_tris = (int)buf.shape(0);

        if (scene)  { rtcReleaseScene(scene);  scene  = nullptr; }
        if (device) { rtcReleaseDevice(device); device = nullptr; }

        device = rtcNewDevice(nullptr);
        if (!device) throw std::runtime_error("rtcNewDevice failed");
        scene  = rtcNewScene(device);
        if (!scene)  throw std::runtime_error("rtcNewScene failed");

        // One triangle geometry
        RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

        float* verts = (float*)rtcSetNewGeometryBuffer(
            geom, RTC_BUFFER_TYPE_VERTEX, 0,
            RTC_FORMAT_FLOAT3, sizeof(float)*3, n_tris*3);
        unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(
            geom, RTC_BUFFER_TYPE_INDEX, 0,
            RTC_FORMAT_UINT3, sizeof(unsigned)*3, n_tris);

        for (int i = 0; i < n_tris; ++i) {
            for (int v = 0; v < 3; ++v) {
                verts[(i*3+v)*3+0] = buf(i, v, 0);
                verts[(i*3+v)*3+1] = buf(i, v, 1);
                verts[(i*3+v)*3+2] = buf(i, v, 2);
            }
            indices[i*3+0] = i*3+0;
            indices[i*3+1] = i*3+1;
            indices[i*3+2] = i*3+2;
        }

        rtcCommitGeometry(geom);
        rtcAttachGeometry(scene, geom);
        rtcReleaseGeometry(geom);
        rtcCommitScene(scene);
    }

    // Set beam directions (angle_start, angle_end, n_beams, range_max)
    void set_beams(float angle_start, float angle_end, int nb, float rmax) {
        n_beams   = nb;
        range_max = rmax;
        base_dx.resize(nb);
        base_dy.resize(nb);
        float step = (angle_end - angle_start) / (nb - 1);
        for (int i = 0; i < nb; ++i) {
            float a = angle_start + i * step;
            base_dx[i] = std::cos(a);
            base_dy[i] = std::sin(a);
        }
    }

    // Per-step cast: rotate directions by theta, fire rays, return range array
    py::array_t<float> cast(float ox, float oy, float theta) {
        if (!scene) throw std::runtime_error("call build_scene() first");
        if (n_beams == 0) throw std::runtime_error("call set_beams() first");

        float ct = std::cos(theta), st = std::sin(theta);

        auto out = py::array_t<float>(n_beams);
        float* ranges = out.mutable_data();

        RTCRayHit rh;
        std::memset(&rh, 0, sizeof(rh));
        rh.ray.org_x = ox;
        rh.ray.org_y = oy;
        rh.ray.org_z = RAY_ORIGIN_Z;
        rh.ray.tnear = TNEAR;
        rh.ray.time  = 0.0f;
        rh.ray.mask  = -1;
        rh.ray.id    = 0;
        rh.ray.flags = 0;

        RTCIntersectArguments args;
        rtcInitIntersectArguments(&args);

        for (int i = 0; i < n_beams; ++i) {
            float dx = ct * base_dx[i] - st * base_dy[i];
            float dy = st * base_dx[i] + ct * base_dy[i];

            rh.ray.dir_x  = dx;
            rh.ray.dir_y  = dy;
            rh.ray.dir_z  = RAY_DIR_Z;
            rh.ray.tfar   = range_max;
            rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;

            rtcIntersect1(scene, &rh, &args);

            ranges[i] = (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID)
                        ? rh.ray.tfar   // Embree writes hit distance into tfar
                        : range_max;
        }
        return out;
    }

    // Packet cast using rtcIntersect8 (8-ray SIMD)
    py::array_t<float> cast_packet8(float ox, float oy, float theta) {
        if (!scene) throw std::runtime_error("call build_scene() first");
        if (n_beams == 0) throw std::runtime_error("call set_beams() first");

        float ct = std::cos(theta), st = std::sin(theta);

        auto out = py::array_t<float>(n_beams);
        float* ranges = out.mutable_data();

        // Align packet structs
        alignas(32) RTCRayHit8 rh8;
        alignas(32) int valid8[8];

        RTCIntersectArguments args;
        rtcInitIntersectArguments(&args);

        int n_full = n_beams / 8;
        int n_rem  = n_beams % 8;

        auto fill_packet = [&](int base, int count) {
            for (int k = 0; k < 8; ++k) {
                if (k < count) {
                    int i = base + k;
                    float dx = ct * base_dx[i] - st * base_dy[i];
                    float dy = st * base_dx[i] + ct * base_dy[i];
                    valid8[k]       = -1;
                    rh8.ray.org_x[k]= ox;
                    rh8.ray.org_y[k]= oy;
                    rh8.ray.org_z[k]= RAY_ORIGIN_Z;
                    rh8.ray.dir_x[k]= dx;
                    rh8.ray.dir_y[k]= dy;
                    rh8.ray.dir_z[k]= RAY_DIR_Z;
                    rh8.ray.tnear[k]= TNEAR;
                    rh8.ray.tfar[k] = range_max;
                    rh8.ray.time[k] = 0.0f;
                    rh8.ray.mask[k] = -1;
                    rh8.ray.id[k]   = 0;
                    rh8.ray.flags[k]= 0;
                    rh8.hit.geomID[k]= RTC_INVALID_GEOMETRY_ID;
                } else {
                    valid8[k] = 0;  // inactive lane
                }
            }
        };

        for (int p = 0; p < n_full; ++p) {
            fill_packet(p * 8, 8);
            rtcIntersect8(valid8, scene, &rh8, &args);
            for (int k = 0; k < 8; ++k) {
                ranges[p * 8 + k] = (rh8.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID)
                                     ? rh8.ray.tfar[k]
                                     : range_max;
            }
        }
        if (n_rem > 0) {
            fill_packet(n_full * 8, n_rem);
            rtcIntersect8(valid8, scene, &rh8, &args);
            for (int k = 0; k < n_rem; ++k) {
                ranges[n_full * 8 + k] = (rh8.hit.geomID[k] != RTC_INVALID_GEOMETRY_ID)
                                          ? rh8.ray.tfar[k]
                                          : range_max;
            }
        }
        return out;
    }
};

// ---------------------------------------------------------------------------
// pybind11 module
// ---------------------------------------------------------------------------
PYBIND11_MODULE(lidar2d_embree, m) {
    m.doc() = "Direct Embree4 LiDAR 2D ray caster — minimal Python overhead";

    py::class_<Lidar2DEmbree>(m, "Lidar2DEmbree")
        .def(py::init<>())
        .def("build_scene", &Lidar2DEmbree::build_scene,
             py::arg("triangles"),
             "Build BVH from float32 triangle soup, shape [N,3,3].")
        .def("set_beams", &Lidar2DEmbree::set_beams,
             py::arg("angle_start"), py::arg("angle_end"),
             py::arg("n_beams"), py::arg("range_max"),
             "Configure beam angles (radians) and max range.")
        .def("cast", &Lidar2DEmbree::cast,
             py::arg("ox"), py::arg("oy"), py::arg("theta"),
             "Fire all beams from (ox,oy) at heading theta. Returns float32 range array.")
        .def("cast_packet8", &Lidar2DEmbree::cast_packet8,
             py::arg("ox"), py::arg("oy"), py::arg("theta"),
             "Same as cast() but uses rtcIntersect8 AVX2 8-ray packets.");
}
