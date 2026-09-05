#pragma once

#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <algorithm>

// Check for SIMD support
#if defined(__AVX2__) || (defined(_MSC_VER) && defined(__AVX2__))
    #define VAMP_USE_AVX2 1
    #include <immintrin.h>
#elif defined(__SSE4_1__) || (defined(_MSC_VER) && !defined(_M_ARM))
    #define VAMP_USE_SSE 1
    #include <smmintrin.h>
#else
    #define VAMP_USE_SCALAR 1
#endif

namespace vamp_r2000ic {

struct Point3 {
    float x, y, z;
    Point3() : x(0.f), y(0.f), z(0.f) {}
    Point3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Sphere {
    float x, y, z;
    float r; // radius
    Sphere() : x(0.f), y(0.f), z(0.f), r(0.f) {}
    Sphere(float x_, float y_, float z_, float r_) : x(x_), y(y_), z(z_), r(r_) {}
};

struct AABB {
    Point3 min;
    Point3 max;
    AABB() = default;
    AABB(const Point3& min_, const Point3& max_) : min(min_), max(max_) {}
};

/**
 * @brief SIMD-optimized distance square test between a sphere and an Axis-Aligned Bounding Box (AABB).
 * Returns true if the sphere collides with the AABB (distance <= sphere.r).
 */
inline bool check_sphere_aabb_collision(const Sphere& s, const AABB& box, float margin = 0.0f) {
    float total_r = s.r + margin;
    float total_r2 = total_r * total_r;

#if defined(VAMP_USE_AVX2)
    // AVX2 implementation: clamp sphere center to box [min, max], compute squared distance
    __m128 sc = _mm_set_ps(0.0f, s.z, s.y, s.x);
    __m128 bmin = _mm_set_ps(0.0f, box.min.z, box.min.y, box.min.x);
    __m128 bmax = _mm_set_ps(0.0f, box.max.z, box.max.y, box.max.x);

    // closest_pt = max(min, min(sc, max))
    __m128 clamped = _mm_max_ps(bmin, _mm_min_ps(sc, bmax));
    __m128 diff = _mm_sub_ps(sc, clamped);
    __m128 sq = _mm_mul_ps(diff, diff);

    // Sum x, y, z
    alignas(16) float sq_arr[4];
    _mm_store_ps(sq_arr, sq);
    float d2 = sq_arr[0] + sq_arr[1] + sq_arr[2];

    return d2 <= total_r2;
#else
    // Scalar fallback
    float cx = std::max(box.min.x, std::min(s.x, box.max.x));
    float cy = std::max(box.min.y, std::min(s.y, box.max.y));
    float cz = std::max(box.min.z, std::min(s.z, box.max.z));

    float dx = s.x - cx;
    float dy = s.y - cy;
    float dz = s.z - cz;
    float d2 = dx * dx + dy * dy + dz * dz;

    return d2 <= total_r2;
#endif
}

/**
 * @brief Check collision between two spheres.
 */
inline bool check_sphere_sphere_collision(const Sphere& s1, const Sphere& s2, float margin = 0.0f) {
    float dx = s1.x - s2.x;
    float dy = s1.y - s2.y;
    float dz = s1.z - s2.z;
    float d2 = dx * dx + dy * dy + dz * dz;
    float r_sum = s1.r + s2.r + margin;
    return d2 <= (r_sum * r_sum);
}

} // namespace vamp_r2000ic
