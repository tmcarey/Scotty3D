
#include "../rays/bsdf.h"
#include "../util/rand.h"

namespace PT {

static Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 5
    // Return reflection of dir about the surface normal (0,1,0).
    return Vec3(-dir.x, fabsf(dir.y), -dir.z);
}

static Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 5
    // Use Snell's Law to refract out_dir through the surface.
    // Return the refracted direction. Set was_internal to true if
    // refraction does not occur due to total internal reflection,
    // and false otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.
    out_dir = -out_dir;
    Vec2 tangentDir = Vec2(out_dir.x, out_dir.z).unit();
    float sin_theta_i = Vec2(out_dir.x, out_dir.z).norm();
    float sin_theta_o = 0.0f;
    float ior = index_of_refraction;
    if(out_dir.y < 0.0f){
        //entering surface
        sin_theta_o = sin_theta_i / (ior);
    }else{
        //exiting surface
        sin_theta_o = sin_theta_i * (ior);
        if(sin_theta_o > 1.0f){
            was_internal = true;
            return Vec3{};
        }
    }
    float cos_theta_o = sqrt(1 - (sin_theta_o * sin_theta_o));
    was_internal = false;
    

    return Vec3(tangentDir.x * sin_theta_o, 
                sign(out_dir.y) * cos_theta_o, 
                tangentDir.y * sin_theta_o);
}

Scatter BSDF_Lambertian::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 4

    // Sample the BSDF distribution using the cosine-weighted hemisphere sampler.
    // You can use BSDF_Lambertian::evaluate() to compute attenuation.

    Scatter ret;
    ret.direction = sampler.sample();
    ret.attenuation = evaluate(out_dir, ret.direction);
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo * cos(theta).

    return in_dir.y > 0.0f ? albedo * in_dir.y : Spectrum(0,0,0);
}

float BSDF_Lambertian::pdf(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
    return (in_dir.y) / (3.14159265f);
}

Scatter BSDF_Mirror::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    Scatter ret;
    ret.direction = reflect(out_dir);
    ret.attenuation = reflectance;
    return ret;
}

Scatter BSDF_Glass::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?
    float f = 0;
    float r0 = pow((index_of_refraction - 1.0f)/ (index_of_refraction + 1.0f), 2.0f);
    if(out_dir.y > 0.0f){
        f = r0  + (1 - r0) * pow(1 - out_dir.y, 5.0f);
    }else{
        f = r0  + (1 - r0) * pow(1 + out_dir.y, 5.0f);
    }
    bool shouldReflect = RNG::coin_flip(f); 

    Scatter ret;
    bool was_internal = false;
    ret.direction = shouldReflect ? reflect(out_dir) : refract(out_dir, index_of_refraction, was_internal);
    ret.attenuation = shouldReflect ? reflectance : was_internal ? Spectrum(0,0,0) : transmittance;
    return ret;
}

Scatter BSDF_Refract::scatter(Vec3 out_dir) const {

    // OPTIONAL (PathTracer): Task 5

    // When debugging BSDF_Glass, it may be useful to compare to a pure-refraction BSDF

    Scatter ret;
    bool was_internal = false;
    ret.direction = refract(out_dir, index_of_refraction, was_internal);
    ret.attenuation = was_internal ? Spectrum(0, 0, 0) : transmittance;

    return ret;
}

Spectrum BSDF_Diffuse::emissive() const {
    return radiance;
}

} // namespace PT
