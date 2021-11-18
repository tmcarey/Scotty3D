
#include "../rays/env_light.h"

#include <limits>

namespace PT {

Vec3 Env_Map::sample() const {

    // TODO (PathTracer): Task 7

    // First, implement Samplers::Sphere::Uniform so the following line works.
    // Second, implement Samplers::Sphere::Image and swap to image_sampler

    return image_sampler.sample();
}

float Env_Map::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // First, return the pdf for a uniform spherical distribution.
    // Second, swap to image_sampler.pdf().

    return image_sampler.pdf(dir);
}

Spectrum Env_Map::evaluate(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // Compute emitted radiance along a given direction by finding the corresponding
    // pixels in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 nearest pixels.
    const auto [w, h] = image.dimension();
    float theta = (acos(-dir.y) / M_PI) * h;
    float phi = ((atan2(-dir.z, -dir.x) + M_PI) / (2.0f * M_PI)) * w; 
    float t = theta - floor(theta);
    float s = phi - floor(phi);
    int x1 = (int)floor(phi) % w;
    int x2 = (int)ceil(phi) % w;
    int y1 = (int)floor(theta) % h;
    int y2 = (int)ceil(theta) % h;

    return t * (s * image.at(x1, y1)
                + (1 - s) * image.at(x2, y1))
           + (1 - t) * (s * image.at(x1, y2)
                        + (1 - s) * image.at(x2, y2));
}

Vec3 Env_Hemisphere::sample() const {
    return sampler.sample();
}

float Env_Hemisphere::pdf(Vec3 dir) const {
    return 1.0f / (2.0f * PI_F);
}

Spectrum Env_Hemisphere::evaluate(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Vec3 Env_Sphere::sample() const {
    return sampler.sample();
}

float Env_Sphere::pdf(Vec3 dir) const {
    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Sphere::evaluate(Vec3) const {
    return radiance;
}

} // namespace PT
