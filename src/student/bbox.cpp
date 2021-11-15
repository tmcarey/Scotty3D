
#include "../lib/mathlib.h"
#include "debug.h"

void swap(float *x, float *y){
    float temp = *x;
    *x = *y;
    *y = temp;
}

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.
    float tmin = (min.x - ray.point.x) / ray.dir.x;
    float tmax = (max.x - ray.point.x) / ray.dir.x;

    if (tmin > tmax) swap(&tmin, &tmax);

    float tymin = (min.y - ray.point.y) / ray.dir.y;
    float tymax = (max.y - ray.point.y) / ray.dir.y;

    if (tymin > tymax) swap(&tymin, &tymax);

    if((tmin > tymax) || (tymin > tmax)){
        return false;
    }

    tmin = std::max(tymin, tmin);
    tmax = std::min(tymax, tmax);

    float tzmin = (min.z - ray.point.z) / ray.dir.z;
    float tzmax = (max.z - ray.point.z) / ray.dir.z;

    if (tzmin > tzmax) swap(&tzmin, &tzmax);

    if((tmin > tzmax) || (tzmin > tmax)){
        return false;
    }

    tmin = std::max(tzmin, tmin);
    tmax = std::min(tzmax, tmax);

    times.x = 0;
    times.y = 0;

    return true;
}
