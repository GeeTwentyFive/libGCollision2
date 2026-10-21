#include "../libGCollision2.hpp"

#include <iostream>

/*
Often when working with AABBs you'll
have them nested within some kind of
data structure (representing some
kind of "object" in your "world"/"scene")
which there are multiple instances of in
an array.

A common operation is to figure out which
instance of those within that array are
hit by a ray.

You *could* write your own loops/function
which goes through each instance one-by-one
and calls `IntersectRayAABB()` along with
a pointer to a `distance` float for
manual comparison.

However, that is unnecessary in this case,
since libGCollision2 offers a function
which does that for you (*without* you
having to costily pull out every instance's
AABB into their own collection to pass as an arg
(thanks to pointer arithmetic; offset+stride)):
`IntersectRayAABBs()`
*/

int main() {
        struct ExampleObject {
                int example_data;
                gcollision2::AABB aabb;
                int example_data_2;
        };

        ExampleObject example_objects[3] = {
                {0, {{-1, -1, -1}, {1, 1, 1}}, 0},
                {0, {{-1, -1, -2}, {1, 1, 0}}, 0},
                {0, {{-1, -1, -3}, {1, 1, -1}}, 0}
        };

        float ray_hit_distance;
        float3 ray_hit_point;
        float3 ray_hit_normal;
        bool ray_inside;
        int64_t hit_aabb_index = gcollision2::IntersectRayAABBs(
                {0, 0, 4}, {0, 0, -1},
                example_objects, 3, offsetof(ExampleObject, aabb), sizeof(ExampleObject),
                &ray_hit_distance, &ray_hit_point, &ray_hit_normal, &ray_inside
        );
        if (hit_aabb_index != -1) {
                std::cout << "Ray hit!" << std::endl;
                std::cout << "Hit AABB index: " << hit_aabb_index << std::endl;
                std::cout << "Ray hit distance: " << ray_hit_distance << std::endl;
                std::cout << "Ray hit point X: " << ray_hit_point.x << std::endl;
                std::cout << "Ray hit point Y: " << ray_hit_point.y << std::endl;
                std::cout << "Ray hit point Z: " << ray_hit_point.z << std::endl;
                std::cout << "Ray hit normal X: " << ray_hit_normal.x << std::endl;
                std::cout << "Ray hit normal Y: " << ray_hit_normal.y << std::endl;
                std::cout << "Ray hit normal Z: " << ray_hit_normal.z << std::endl;
                std::cout << "Ray inside?: " << ray_inside << std::endl;
        }
        else std::cout << "No ray hit!" << std::endl;

        return 0;
}