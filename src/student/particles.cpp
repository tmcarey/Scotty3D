
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Scene_Particles::Particle::update(const PT::Object& scene, float dt, float radius) {

    // TODO(Animation): Task 4

    // Compute the trajectory of this particle for the next dt seconds.
    float remaining = dt;

    // (1) Build a ray representing the particle's path if it travelled at constant velocity.
    for(uint i = 0; i < 5 && remaining > 0.0001f; i++){
        Ray trajectory;
        trajectory.point = pos;
        trajectory.dir = velocity.unit();

        // (2) Intersect the ray with the scene and account for collisions. Be careful when placing
        // collision points using the particle radius. Move the particle to its next position.
        PT::Trace hit = scene.hit(trajectory);

        Vec3 o = pos;
        Vec3 v = velocity.unit();
        Vec3 p = hit.position;
        Vec3 n = hit.normal;
        if(dot(v,n) < 0.0f){
            n = -n;
        }
        float dist = ((dot(p,n) - dot(o,n) - (radius/ 2.0f)) / dot(v, n));
        if(!hit.hit){
            dist = remaining * velocity.norm();
        }
        if(dist >= remaining * velocity.norm()){
            pos = trajectory.at(remaining * velocity.norm());
            velocity += (remaining) * acceleration;
            break;
        }else{
            pos = trajectory.at(dist);
            velocity = (v - 2 * dot(v, n) * n) * velocity.norm();
            velocity += (dist / velocity.norm()) * acceleration;
            remaining -= dist;
        }
    }
    

    // (3) Account for acceleration due to gravity.

    // (4) Repeat until the entire time step has been consumed.

    // (5) Decrease the particle's age and return whether it should die.
    age -= dt;

    return age >= 0;
}
