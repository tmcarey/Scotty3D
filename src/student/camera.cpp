
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    float y = atan((3.14159265f / 180.0f) * (Camera::vert_fov));
    float x = Camera::aspect_ratio * y;
    float z = -1.0f;
    Vec3 vec11(x, y, z);
    Vec3 vec10(x, -y, z);
    Vec3 vec01(-x, y, z);
    Vec3 vec00(-x, -y, z);

    float t = screen_coord.x;
    float s = screen_coord.y;
    Vec3 finalDir = t * (s * vec11 + (1 - s) * vec10) + (1 - t) * (s * vec01 + (1 - s) * vec00);
    Ray r(Vec3(0,0,0), finalDir.normalize());
    r.transform(Camera::iview);
    return r;
}
