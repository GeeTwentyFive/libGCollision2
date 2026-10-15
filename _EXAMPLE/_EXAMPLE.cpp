#include "../libGCollision2.hpp"

#include <iostream>

int main() {
        gcollision2::AABB box = {{-1, -1, -1}, {1, 1, 1}};

        if (box.IntersectsAABB(gcollision2::AABB{{0, 0, 0}, {2, 2, 2}})) std::cout << "Box intersect!" << std::endl;
        else std::cout << "No box intersect!" << std::endl;

        std::cout << std::endl;

        float ray_hit_distance;
        float3 ray_hit_point;
        float3 ray_hit_normal;
        if (gcollision2::IntersectRayAABB(
                {0, 0, 4}, {0, 0, -1},
                box,
                &ray_hit_distance, &ray_hit_point, &ray_hit_normal
        )) {
                std::cout << "Ray hit!" << std::endl;
                std::cout << "Ray hit distance: " << ray_hit_distance << std::endl;
                std::cout << "Ray hit point X: " << ray_hit_point.x << std::endl;
                std::cout << "Ray hit point Y: " << ray_hit_point.y << std::endl;
                std::cout << "Ray hit point Z: " << ray_hit_point.z << std::endl;
                std::cout << "Ray hit normal X: " << ray_hit_normal.x << std::endl;
                std::cout << "Ray hit normal Y: " << ray_hit_normal.y << std::endl;
                std::cout << "Ray hit normal Z: " << ray_hit_normal.z << std::endl;
        }
        else std::cout << "No ray hit!" << std::endl;

        return 0;
}