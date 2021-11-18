
#include "../rays/samplers.h"
#include "../util/rand.h"

namespace Samplers {

Vec2 Rect::sample() const {

    // TODO (PathTracer): Task 1

    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()

    return Vec2(RNG::unit() * size.x, RNG::unit() * size.y);
}

Vec3 Sphere::Uniform::sample() const {

    // TODO (PathTracer): Task 7

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    Vec3 out = Vec3(xs, ys, zs);

    if(RNG::coin_flip(0.5f)){
        out = -out;
    }

    return out;
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
    total = 0;
    for(size_t wi = 0; wi < w; wi++){
        for(size_t hi = 0; hi < h; hi++){
            total += image.at(wi, hi).luma();
        }
    }

    float cumulPdf = 0;
    _cdf.resize(w * h);
    _pdf.resize(w * h);
    for(size_t i = 0; i < (w * h); i++){
        _cdf[i] = cumulPdf;
        float luma = image.at(i).luma();
        _pdf[i] = luma / total;
        cumulPdf += _pdf[i];
    }
}

Vec3 Sphere::Image::sample() const {

    // TODO (PathTracer): Task 7

    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound
    size_t idx = std::upper_bound(_cdf.begin(), _cdf.end(), RNG::unit()) - _cdf.begin();


    float theta = ((float)(idx / w) / (float)h) * M_PI;
    float phi = ((float)(idx % w) / (float)w) * 2.0f * M_PI;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

float Sphere::Image::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // What is the PDF of this distribution at a particular direction?
    float theta = acos(-dir.y);
    float hi = (theta / M_PI) * h;
    float wi = ((atan2(-dir.z, -dir.x) + M_PI) / (2.0f * M_PI)) * w; 

    return _pdf[(w * hi) + wi] * w * h / (2.0f * M_PI * M_PI * sin(theta));
}

Vec3 Point::sample() const {
    return point;
}

Vec3 Triangle::sample() const {
    float u = std::sqrt(RNG::unit());
    float v = RNG::unit();
    float a = u * (1.0f - v);
    float b = u * v;
    return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

Vec3 Hemisphere::Uniform::sample() const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

Vec3 Hemisphere::Cosine::sample() const {

    float phi = RNG::unit() * 2.0f * PI_F;
    float cos_t = std::sqrt(RNG::unit());

    float sin_t = std::sqrt(1 - cos_t * cos_t);
    float x = std::cos(phi) * sin_t;
    float z = std::sin(phi) * sin_t;
    float y = cos_t;

    return Vec3(x, y, z);
}

} // namespace Samplers
