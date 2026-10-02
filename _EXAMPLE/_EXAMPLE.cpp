#include "../include/libGCollision2.hpp"

#include <iostream>

int main() {
        gcollision2::AABB box = {{-1, -1, -1}, {1, 1, 1}};

        if (box.Intersects(gcollision2::AABB{{0, 0, 0}, {2, 2, 2}})) std::cout << "Box intersect!" << std::endl;
        else std::cout << "No box intersect!" << std::endl;

        std::cout << std::endl;

        gcollision2::RayHitExtraInfo ray_hit_info;
        if (gcollision2::IntersectRayAABB(
                {0, 0, 4}, {0, 0, -1},
                box, &ray_hit_info
        )) {
                std::cout << "Ray hit!" << std::endl;
                std::cout << "Ray hit distance: " << ray_hit_info.distance << std::endl;
                std::cout << "Ray hit point X: " << ray_hit_info.point.x << std::endl;
                std::cout << "Ray hit point Y: " << ray_hit_info.point.y << std::endl;
                std::cout << "Ray hit point Z: " << ray_hit_info.point.z << std::endl;
                std::cout << "Ray hit normal X: " << ray_hit_info.normal.x << std::endl;
                std::cout << "Ray hit normal Y: " << ray_hit_info.normal.y << std::endl;
                std::cout << "Ray hit normal Z: " << ray_hit_info.normal.z << std::endl;
        }
        else std::cout << "No ray hit!" << std::endl;

        return 0;
}