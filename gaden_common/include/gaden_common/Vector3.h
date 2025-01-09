#pragma once
#define GLM_FORCE_INLINE
#define GLM_FORCE_XYZW_ONLY
#define GLM_ENABLE_EXPERIMENTAL
#include <gaden_common/third_party/glm/glm/gtx/rotate_vector.hpp>
#include <gaden_common/third_party/glm/glm/common.hpp>
#include <gaden_common/third_party/glm/glm/geometric.hpp>
#include <gaden_common/third_party/glm/glm/vec2.hpp>
#include <gaden_common/third_party/glm/glm/vec3.hpp>

namespace gaden
{
    using Vector2 = glm::vec2;
    using Vector3 = glm::vec3;
    using Vector2i = glm::ivec2;
    using Vector3i = glm::ivec3;
    template <typename Vec>
    inline float length(const Vec& vec)
    {
        return glm::length(vec);
    }

    template <typename Vec>
    inline Vec normalized(const Vec& vec)
    {
        return glm::normalize(vec);
    }

    template <typename Vec>
    inline Vec cross(const Vec& a, const Vec& b)
    {
        return glm::cross(a, b);
    }


    template <typename Vec>
    inline float dot(const Vec& a, const Vec& b)
    {
        return glm::dot(a, b);
    }

    template <typename Vec>
    inline Vec rotate(const Vec& vec, float signedAngleRadians)
    {
        return glm::rotate(vec, signedAngleRadians);
    }

    inline Vector2 transpose(const Vector2& vec)
    {
        return {vec.y, vec.x};
    }

    inline Vector3 WithZ(const Vector2& vec, float z)
    {
        return Vector3(vec.x, vec.y, z);
    }

} // namespace GSL::vmath
