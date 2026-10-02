# Examples

AABB -> AABB intersection:
```c++
bool colliding = gcollision2::AABB{
        {-1, -1, -1},
        {1, 1, 1}
}.Intersects(
        gcollision2::AABB{
                {0, 0, 0},
                {2, 2, 2}
        }
);
```

Ray -> AABB intersection:
```c++
gcollision2::RayHitInfo hit_info = gcollision2::IntersectRayAABB(
        {0, 0, 0},
        {0, 0, -1},
        {{2, 2, -2}, {-2, -2, -3}}
);
```