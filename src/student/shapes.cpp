
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?

    float discr = pow(dot(ray.point, ray.dir), 2.0) - dot(ray.point, ray.point) + (radius * radius);
    if(discr < 0.0f){
        return ret;
    }
    float t1 = -dot(ray.point, ray.dir) - sqrt(discr);
    float t2 = -dot(ray.point, ray.dir) + sqrt(discr);
    float t = std::min(t1, t2);
    ret.position = ray.at(t);
    ret.normal = ret.position.unit();
    if(t != clamp(t, ray.dist_bounds.x, ray.dist_bounds.y)){
        t = std::max(t1,t2);
        ret.position = ray.at(t);
        ret.normal = (ret.position.unit());
        if(t != clamp(t, ray.dist_bounds.x, ray.dist_bounds.y)){
            return ret;
        }
    }
    ret.hit = true;
    ret.distance = t;
    ret.position = ray.at(t);

    return ret;
}

} // namespace PT
