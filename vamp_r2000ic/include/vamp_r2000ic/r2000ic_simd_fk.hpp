#pragma once

#include "simd_defs.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <unordered_map>
#include <string>
#include <algorithm>

namespace vamp_r2000ic {

/**
 * @brief Lightweight 4x4 Transform matrix for SIMD forward kinematics
 */
struct Transform4 {
    float m[16]; // Column-major

    Transform4() {
        setIdentity();
    }

    void setIdentity() {
        for (int i = 0; i < 16; ++i) m[i] = 0.0f;
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }

    static Transform4 Translation(float x, float y, float z) {
        Transform4 t;
        t.m[12] = x;
        t.m[13] = y;
        t.m[14] = z;
        return t;
    }

    static Transform4 RotZ(float theta) {
        Transform4 t;
        float c = std::cos(theta);
        float s = std::sin(theta);
        t.m[0] = c;  t.m[4] = -s;
        t.m[1] = s;  t.m[5] = c;
        return t;
    }

    static Transform4 RotY(float theta) {
        Transform4 t;
        float c = std::cos(theta);
        float s = std::sin(theta);
        t.m[0] = c;   t.m[8] = s;
        t.m[2] = -s;  t.m[10] = c;
        return t;
    }

    static Transform4 RotX(float theta) {
        Transform4 t;
        float c = std::cos(theta);
        float s = std::sin(theta);
        t.m[5] = c;  t.m[9] = -s;
        t.m[6] = s;  t.m[10] = c;
        return t;
    }

    static Transform4 RotAxis(float ux, float uy, float uz, float theta) {
        Transform4 t;
        float c = std::cos(theta);
        float s = std::sin(theta);
        float v = 1.0f - c;

        t.m[0] = ux * ux * v + c;
        t.m[1] = uy * ux * v + uz * s;
        t.m[2] = uz * ux * v - uy * s;
        t.m[3] = 0.0f;

        t.m[4] = ux * uy * v - uz * s;
        t.m[5] = uy * uy * v + c;
        t.m[6] = uz * uy * v + ux * s;
        t.m[7] = 0.0f;

        t.m[8] = ux * uz * v + uy * s;
        t.m[9] = uy * uz * v - ux * s;
        t.m[10] = uz * uz * v + c;
        t.m[11] = 0.0f;

        t.m[12] = 0.0f;
        t.m[13] = 0.0f;
        t.m[14] = 0.0f;
        t.m[15] = 1.0f;
        return t;
    }

    static Transform4 FromXYZRPY(float x, float y, float z, float r, float p, float yaw) {
        Transform4 t = Translation(x, y, z);
        Transform4 rz = RotZ(yaw);
        Transform4 ry = RotY(p);
        Transform4 rx = RotX(r);
        return t * rz * ry * rx;
    }

    Transform4 operator*(const Transform4& o) const {
        Transform4 res;
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                res.m[col * 4 + row] = 
                    m[0 * 4 + row] * o.m[col * 4 + 0] +
                    m[1 * 4 + row] * o.m[col * 4 + 1] +
                    m[2 * 4 + row] * o.m[col * 4 + 2] +
                    m[3 * 4 + row] * o.m[col * 4 + 3];
            }
        }
        return res;
    }

    Point3 transformPoint(const Point3& p) const {
        return Point3(
            m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
            m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
            m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]
        );
    }
};

struct LocalSphere {
    int link_index; // 0: base, 1: J1, 2: J2, 3: J3, 4: J4, 5: J5, 6: J6
    Point3 local_pos;
    float radius;
};

/**
 * @brief Forward Kinematics unrolled and vectorized for 6-DOF industrial robot.
 * Supports dynamic configuration of joint origins from URDF with L1 Cache residency.
 */
class R2000icKinematics {
public:
    static const int NUM_JOINTS = 6;
    static const int NUM_LINKS = 7; // base + 6 joints

    R2000icKinematics() {
        initDefaultOrigins();
        initDefaultSpheres();
    }

    /**
     * @brief Dynamically configures the 6 joint origins (xyz, rpy) and axes from URDF.
     * @param origins_xyz Array of 18 doubles (6 * [x, y, z])
     * @param origins_rpy Array of 18 doubles (6 * [r, p, y])
     * @param axes_xyz Array of 18 doubles (6 * [axis_x, axis_y, axis_z])
     */
    void setJointOrigins(const double origins_xyz[18], const double origins_rpy[18], const double axes_xyz[18]) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            float x = static_cast<float>(origins_xyz[i * 3 + 0]);
            float y = static_cast<float>(origins_xyz[i * 3 + 1]);
            float z = static_cast<float>(origins_xyz[i * 3 + 2]);

            float r = static_cast<float>(origins_rpy[i * 3 + 0]);
            float p = static_cast<float>(origins_rpy[i * 3 + 1]);
            float yaw = static_cast<float>(origins_rpy[i * 3 + 2]);

            origin_transforms_[i] = Transform4::FromXYZRPY(x, y, z, r, p, yaw);

            float ax = static_cast<float>(axes_xyz[i * 3 + 0]);
            float ay = static_cast<float>(axes_xyz[i * 3 + 1]);
            float az = static_cast<float>(axes_xyz[i * 3 + 2]);
            float len = std::sqrt(ax * ax + ay * ay + az * az);
            if (len > 1e-6f) {
                ax /= len; ay /= len; az /= len;
            } else {
                az = 1.0f;
            }
            axes_[i] = Point3(ax, ay, az);
        }
    }

    /**
     * @brief Computes link world transforms from joint angles (radians).
     * @param joints 6-element joint angle vector [J1, J2, J3, J4, J5, J6]
     * @param link_transforms output array of 7 transforms (base_link + J1..J6)
     */
    void computeLinkTransforms(const double joints[6], std::array<Transform4, NUM_LINKS>& link_transforms) const {
        // Base link transform = Identity
        link_transforms[0].setIdentity();

        for (int i = 0; i < NUM_JOINTS; ++i) {
            Transform4 T_rot = Transform4::RotAxis(axes_[i].x, axes_[i].y, axes_[i].z, static_cast<float>(joints[i]));
            Transform4 T_joint = origin_transforms_[i] * T_rot;
            link_transforms[i + 1] = link_transforms[i] * T_joint;
        }
    }

    /**
     * @brief Computes world-space positions for all bounding spheres.
     * @param joints 6-element joint angles
     * @param world_spheres output list of spheres in world coordinates
     * @param sphere_links optional output list of corresponding link indices for each sphere
     */
    void computeWorldSpheres(const double joints[6], std::vector<Sphere>& world_spheres, std::vector<int>* sphere_links = nullptr) const {
        std::array<Transform4, NUM_LINKS> link_transforms;
        computeLinkTransforms(joints, link_transforms);

        size_t total_count = local_spheres_.size();
        for (const auto& pair : attached_spheres_) {
            total_count += pair.second.size();
        }

        world_spheres.resize(total_count);
        if (sphere_links) sphere_links->resize(total_count);

        for (size_t i = 0; i < local_spheres_.size(); ++i) {
            const auto& ls = local_spheres_[i];
            const auto& t = link_transforms[ls.link_index];
            Point3 wp = t.transformPoint(ls.local_pos);
            world_spheres[i] = Sphere(wp.x, wp.y, wp.z, ls.radius);
            if (sphere_links) (*sphere_links)[i] = ls.link_index;
        }

        size_t cur_idx = local_spheres_.size();
        for (const auto& pair : attached_spheres_) {
            for (const auto& ls : pair.second) {
                int link_idx = std::clamp(ls.link_index, 0, NUM_LINKS - 1);
                const auto& t = link_transforms[link_idx];
                Point3 wp = t.transformPoint(ls.local_pos);
                world_spheres[cur_idx] = Sphere(wp.x, wp.y, wp.z, ls.radius);
                if (sphere_links) (*sphere_links)[cur_idx] = link_idx;
                cur_idx++;
            }
        }
    }

    void addAttachedSpheres(const std::string& name, int link_index, const std::vector<Sphere>& spheres) {
        std::vector<LocalSphere> list;
        for (const auto& s : spheres) {
            list.push_back({ link_index, Point3(s.x, s.y, s.z), s.r });
        }
        attached_spheres_[name] = list;
    }

    bool removeAttachedSpheres(const std::string& name) {
        return attached_spheres_.erase(name) > 0;
    }

    void clearAttachedSpheres() {
        attached_spheres_.clear();
    }

    const std::vector<LocalSphere>& getLocalSpheres() const {
        return local_spheres_;
    }

    const std::unordered_map<std::string, std::vector<LocalSphere>>& getAttachedSpheres() const {
        return attached_spheres_;
    }

    const std::array<Transform4, NUM_JOINTS>& getOriginTransforms() const {
        return origin_transforms_;
    }

    const std::array<Point3, NUM_JOINTS>& getAxes() const {
        return axes_;
    }

private:
    std::vector<LocalSphere> local_spheres_;
    std::unordered_map<std::string, std::vector<LocalSphere>> attached_spheres_;
    std::array<Transform4, NUM_JOINTS> origin_transforms_;
    std::array<Point3, NUM_JOINTS> axes_;

    void initDefaultOrigins() {
        // Standard Fanuc R-2000iC/165F defaults
        origin_transforms_[0] = Transform4::Translation(0.0f, 0.0f, 0.67f);
        axes_[0] = Point3(0.0f, 0.0f, 1.0f);

        origin_transforms_[1] = Transform4::Translation(0.312f, 0.0f, 0.0f);
        axes_[1] = Point3(0.0f, 1.0f, 0.0f);

        origin_transforms_[2] = Transform4::Translation(0.0f, 0.0f, 1.075f);
        axes_[2] = Point3(0.0f, -1.0f, 0.0f);

        origin_transforms_[3] = Transform4::Translation(0.0f, 0.0f, 0.225f);
        axes_[3] = Point3(-1.0f, 0.0f, 0.0f);

        origin_transforms_[4] = Transform4::Translation(1.28f, 0.0f, 0.0f);
        axes_[4] = Point3(0.0f, -1.0f, 0.0f);

        origin_transforms_[5] = Transform4::Translation(0.0f, 0.0f, 0.0f);
        axes_[5] = Point3(-1.0f, 0.0f, 0.0f);
    }

    void initDefaultSpheres() {
        // Base link (link 0) - FOAM Medial Axis Spheres from base.dae
        local_spheres_.push_back({0, Point3(-0.3132f, +0.0030f, +0.1198f), 0.3642f});
        local_spheres_.push_back({0, Point3(+0.0817f, +0.1050f, +0.1292f), 0.3609f});
        local_spheres_.push_back({0, Point3(+0.0720f, -0.1298f, +0.1338f), 0.3532f});
        local_spheres_.push_back({0, Point3(-0.5684f, +0.1284f, +0.0892f), 0.1038f});

        // J1 link (link 1) - FOAM Medial Axis Spheres from j1.dae
        local_spheres_.push_back({1, Point3(+0.2581f, -0.0248f, -0.0696f), 0.3457f});
        local_spheres_.push_back({1, Point3(-0.2134f, +0.1796f, -0.0741f), 0.3517f});
        local_spheres_.push_back({1, Point3(-0.0140f, -0.0517f, -0.2107f), 0.3242f});
        local_spheres_.push_back({1, Point3(+0.0784f, +0.2003f, -0.1882f), 0.3344f});

        // J2 link (link 2) - FOAM Medial Axis Spheres from j2.dae (Upper Arm)
        local_spheres_.push_back({2, Point3(-0.0349f, +0.1926f, +0.0262f), 0.2646f});
        local_spheres_.push_back({2, Point3(+0.0756f, +0.2324f, +0.5343f), 0.2474f});
        local_spheres_.push_back({2, Point3(+0.0307f, +0.3011f, +1.0455f), 0.2195f});
        local_spheres_.push_back({2, Point3(+0.0475f, +0.2671f, +0.8351f), 0.2067f});
        local_spheres_.push_back({2, Point3(-0.0603f, +0.2827f, +1.1263f), 0.1677f});
        local_spheres_.push_back({2, Point3(+0.0582f, +0.1995f, +0.2143f), 0.2344f});

        // J3 link (link 3) - FOAM Elbow & Sleek Forearm Shaft Spheres
        // Elbow body
        local_spheres_.push_back({3, Point3(+0.0600f, +0.0538f, +0.0373f), 0.3000f});
        local_spheres_.push_back({3, Point3(+0.1302f, +0.0110f, +0.1941f), 0.2650f});
        local_spheres_.push_back({3, Point3(+0.2450f, +0.2341f, +0.1791f), 0.1450f});
        local_spheres_.push_back({3, Point3(+0.1631f, +0.3777f, +0.1024f), 0.1350f});
        // Forearm shaft (tightly wraps cylinder tube)
        local_spheres_.push_back({3, Point3(+0.4500f, +0.0258f, +0.2404f), 0.1250f});
        local_spheres_.push_back({3, Point3(+0.6000f, +0.0011f, +0.2394f), 0.1100f});
        local_spheres_.push_back({3, Point3(+0.7500f, -0.0240f, +0.2387f), 0.1250f});
        local_spheres_.push_back({3, Point3(+0.9000f, +0.0004f, +0.2242f), 0.1320f});
        local_spheres_.push_back({3, Point3(+1.0200f, +0.0000f, +0.2250f), 0.1250f});

        // J4 link (link 4) - Tight Forearm Wrist Roll Sleeve Spheres
        local_spheres_.push_back({4, Point3(+1.0900f, +0.0000f, +0.0000f), 0.1120f});
        local_spheres_.push_back({4, Point3(+1.1700f, +0.0017f, +0.0001f), 0.0980f});
        local_spheres_.push_back({4, Point3(+1.2500f, -0.0158f, +0.0005f), 0.1110f});
        local_spheres_.push_back({4, Point3(+1.3300f, -0.0277f, +0.0000f), 0.1070f});

        // J5 link (link 5) - FOAM Medial Axis Spheres from j5.dae (Wrist Tilt)
        local_spheres_.push_back({5, Point3(+0.0974f, -0.0067f, -0.0000f), 0.1421f});
        local_spheres_.push_back({5, Point3(+0.0000f, -0.0790f, +0.0000f), 0.1185f});
        local_spheres_.push_back({5, Point3(+0.0013f, +0.0626f, +0.0001f), 0.0935f});

        // J6 link (link 6) - FOAM Medial Axis Spheres from j6.dae (Flange Faceplate)
        local_spheres_.push_back({6, Point3(+0.1951f, -0.0191f, -0.0018f), 0.0828f});
        local_spheres_.push_back({6, Point3(+0.1947f, +0.0165f, +0.0030f), 0.0838f});
    }
};

} // namespace vamp_r2000ic
