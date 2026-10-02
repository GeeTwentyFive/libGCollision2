// https://github.com/GeeTwentyFive/libGCollision2
#pragma once

#include "linalg/linalg.h"
using namespace linalg::aliases;

#include <limits>

namespace gcollision2 {
struct AABB {
        float3 min;
        float3 max;
        float3 GetSize() const {return max - min;}
        float3 GetHalfExtent() const {return GetSize()/2.0f;}
        float3 GetCenterPos() const {return (min+max)/2.0f;}
        void SetCenterPos(float3 new_pos) {float3 half_extent = GetHalfExtent(); min = new_pos - half_extent; max = new_pos + half_extent;}
        float GetSurfaceArea() const {float3 size = GetSize(); return 2.0f * (size.x*size.y + size.y*size.z + size.z*size.x);}
        bool ContainsPoint(const float3& p) const {return (
                (p.x >= min.x && p.x <= max.x) &&
                (p.y >= min.y && p.y <= max.y) &&
                (p.z >= min.z && p.z <= max.z)
        );}
        bool Intersects(const AABB& b) const {return (
                (min.x <= b.max.x && max.x >= b.min.x) &&
                (min.y <= b.max.y && max.y >= b.min.y) &&
                (min.z <= b.max.z && max.z >= b.min.z)
        );}
        float GetPenetrationDepth(const AABB& b) const { // Only valid if Intersects(b) == true
                float px = std::min(max.x, b.max.x) - std::max(min.x, b.min.x);
                float py = std::min(max.y, b.max.y) - std::max(min.y, b.min.y);
                float pz = std::min(max.z, b.max.z) - std::max(min.z, b.min.z);
                return std::min(std::min(px, py), pz);
        }
        float3 GetCollisionNormal(const AABB& b) const { // Only valid if Intersects(b) == true
                float px = std::min(max.x, b.max.x) - std::max(min.x, b.min.x);
                float py = std::min(max.y, b.max.y) - std::max(min.y, b.min.y);
                float pz = std::min(max.z, b.max.z) - std::max(min.z, b.min.z);

                float3 center = GetCenterPos();
                float3 b_center = b.GetCenterPos();

                if (px < py && px < pz) return float3{((center.x - b_center.x) < 0.0f) ? -1.0f : 1.0f, 0.0f, 0.0f};
                if (py < pz) return float3{0, ((center.y - b_center.y) < 0.0f) ? -1.0f : 1.0f, 0.0f};
                return float3{0, 0, ((center.z - b_center.z) < 0.0f) ? -1.0f : 1.0f};
        }
};

struct RayHitExtraInfo {
        float distance = 0.0f;
        float3 point = float3{0.0f, 0.0f, 0.0f};
        float3 normal = float3{0.0f, 0.0f, 0.0f};
};
bool IntersectRayAABB(  // returns `true` if ray intersects AABB
        const float3& ray_origin,
        const float3& ray_direction,
        const AABB& target,
        gcollision2::RayHitExtraInfo* OUT_extra_hit_info = nullptr  // optional extra hit info
) {
        if (ray_direction.x == 0.0f && (ray_origin.x < target.min.x || ray_origin.x > target.max.x)) { return false; }
        if (ray_direction.y == 0.0f && (ray_origin.y < target.min.y || ray_origin.y > target.max.y)) { return false; }
        if (ray_direction.z == 0.0f && (ray_origin.z < target.min.z || ray_origin.z > target.max.z)) { return false; }

        float t_min_x, t_max_x;
        if (ray_direction.x == 0.0f) {
                t_min_x = std::numeric_limits<float>::lowest();
                t_max_x = std::numeric_limits<float>::max();
        } else if (ray_direction.x < 0.0f) {
                t_min_x = (target.max.x - ray_origin.x) / ray_direction.x;
                t_max_x = (target.min.x - ray_origin.x) / ray_direction.x;
        } else {
                t_min_x = (target.min.x - ray_origin.x) / ray_direction.x;
                t_max_x = (target.max.x - ray_origin.x) / ray_direction.x;
        }

        float t_min_y, t_max_y;
        if (ray_direction.y == 0.0f) {
                t_min_y = std::numeric_limits<float>::lowest();
                t_max_y = std::numeric_limits<float>::max();
        } else if (ray_direction.y < 0.0f) {
                t_min_y = (target.max.y - ray_origin.y) / ray_direction.y;
                t_max_y = (target.min.y - ray_origin.y) / ray_direction.y;
        } else {
                t_min_y = (target.min.y - ray_origin.y) / ray_direction.y;
                t_max_y = (target.max.y - ray_origin.y) / ray_direction.y;
        }

        float t_min_z, t_max_z;
        if (ray_direction.z == 0.0f) {
                t_min_z = std::numeric_limits<float>::lowest();
                t_max_z = std::numeric_limits<float>::max();
        } else if (ray_direction.z < 0.0f) {
                t_min_z = (target.max.z - ray_origin.z) / ray_direction.z;
                t_max_z = (target.min.z - ray_origin.z) / ray_direction.z;
        } else {
                t_min_z = (target.min.z - ray_origin.z) / ray_direction.z;
                t_max_z = (target.max.z - ray_origin.z) / ray_direction.z;
        }

        float t_min = std::max( std::max(t_min_x, t_min_y), t_min_z );
        float t_max = std::min( std::min(t_max_x, t_max_y),  t_max_z );

        if (
                (t_max < 0.0f) ||  // AABB behind ray
                (t_min > t_max)  // No hit
        ) { return false; }

        if (OUT_extra_hit_info == nullptr) return true;

        RayHitExtraInfo extra_hit_info{};
        bool inside_box = (
                (ray_origin.x > target.min.x && ray_origin.x < target.max.x) &&
                (ray_origin.y > target.min.y && ray_origin.y < target.max.y) &&
                (ray_origin.z > target.min.z && ray_origin.z < target.max.z)
        );
        float hit_t = (inside_box ? t_max : t_min);  // If ray is inside box, then t_max (the other point on line (expressed as number representing distance along ray from ray_origin toward ray_direction (works even if not unit-length or normalized to unit-length))) is hit, else it's just t_min as usual
        extra_hit_info.distance = hit_t * linalg::length(ray_direction);  // Multiplied by length of ray_direction, in case ray_direction is not normalized, to scale to world scale
        extra_hit_info.point = ray_origin + (ray_direction * hit_t);
        extra_hit_info.normal = float3{0, 0, 0};  // Since target is an AABB (*Axis-Aligned* Bounding Box) one can just set the same axis on the normal vector as the hit side's axis (is perpendicular to the face of the hit side) to either -1 or 1, depending on if the ray direction on that axis is positive or negative respectively:
        if ((inside_box ? t_max_x : t_min_x) == hit_t) extra_hit_info.normal.x = (ray_direction.x > 0.0f) ? -1.0f : 1.0f;
        else if ((inside_box ? t_max_y : t_min_y) == hit_t) extra_hit_info.normal.y = (ray_direction.y > 0.0f) ? -1.0f : 1.0f;
        else extra_hit_info.normal.z = (ray_direction.z > 0.0f) ? -1.0f : 1.0f;
        if (inside_box) extra_hit_info.normal = -extra_hit_info.normal;
        *OUT_extra_hit_info = extra_hit_info;
        return true;
}
}



/*
Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/