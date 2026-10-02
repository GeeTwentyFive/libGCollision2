# Examples

AABB -> AABB intersection:
```c++
bool intersects = gcollision2::AABB{
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
bool hit = gcollision2::IntersectRayAABB(
        {0, 0, 0},
        {0, 0, -1},
        {{2, 2, -2}, {-2, -2, -3}}
);
```