
#include "../scene/skeleton.h"
#include "../rays/pathtracer.h"

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end
    float t = dot(point - start, (end - start).unit());
    if (t >= (end-start).norm()){
        return end;
    }else if (t <= 0.0f){
        return start;
    }else {
        return ((end - start).unit() * t) + start;
    }
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    Mat4 transform = Mat4();
    const Joint *j = this;
    while(!j->is_root())
    {   
        Mat4 translation = Mat4::translate(j->parent->extent);
        transform = translation * transform;
        j = j->parent;
    }
    return transform;
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos
    const Joint *j = this;
    Mat4 transform = Mat4::euler(j->pose);
    while(!j->is_root())
    {   
        Mat4 rotation = Mat4::euler(j->parent->pose);
        Mat4 translation = Mat4::translate(j->parent->extent);
        transform = rotation * translation * transform;
        j = j->parent;
    }
    return transform;
}

Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    return joint_to_bind(j) * j->extent;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.
    return joint_to_posed(j) * j->extent;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.
    return Mat4::translate(base_pos) * j->joint_to_bind();
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.
    return  Mat4::translate(base_pos) * j->joint_to_posed();
}

void Skeleton::find_joints(const GL::Mesh& mesh, std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping: vertex index -> list of joints that should effect the vertex.
    // A joint should effect a vertex if it is within Joint::radius distance of the
    // bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    map.resize(verts.size());

    for(uint i = 0; i < verts.size(); i++){
        map[i].resize(0);
    }
    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for_joints([&](Joint* j) {
        // What vertices does joint j effect?
        Vec3 jointStart = joint_to_bind(j) * Vec3(0, 0, 0);
        Vec3 jointEnd = (joint_to_bind(j) * j->extent);
        for(uint i = 0; i < verts.size(); i++){
            Vec3 point = verts[i].pos;
            Vec3 pointOnJoint = closest_on_line_segment(jointStart, jointEnd, point);
            if(((point - pointOnJoint).norm()) <= j->radius){
                map[i].push_back(j);
            }
        }
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::vector<std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();
    std::vector<GL::Mesh::Vert> newVerts;
    newVerts.resize(verts.size());

    for(size_t i = 0; i < verts.size(); i++) {
        Vec3 sum = Vec3(0);
        float denom = 0;
        // Skin vertex i. Note that its position is given in object bind space.
        if(map[i].size() < 1){
            sum = verts[i].pos;
            denom = 1.0f;
        }
        for(auto jit = map[i].begin(); jit < map[i].end(); jit++){
            Joint *j = *jit;
            Vec3 jointStart = joint_to_bind(j) * Vec3(0);
            Vec3 jointEnd = joint_to_bind(j) * j->extent;
            Vec3 pointOnJoint = closest_on_line_segment(jointStart, jointEnd, verts[i].pos);
            Mat4 bindToPose = joint_to_posed(j) * joint_to_bind(j).inverse();
            float distInv = 1.0f / (verts[i].pos - pointOnJoint).norm();
            sum += distInv * (bindToPose * verts[i].pos);
            denom += distInv;
        }
        newVerts[i] = verts[i];
        newVerts[i].pos = sum / denom;
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(newVerts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.
    // x gradient
    if(!is_root()){
        parent->compute_gradient(target, current);
    }
    Vec3 delta = current - target;
    Vec3 Jx = cross(joint_to_posed() * Vec3(1, 0, 0), current - (joint_to_posed() * extent));
    float x = dot(Jx, delta);
    Vec3 Jy = cross(joint_to_posed() * Vec3(0, 1, 0), current - (joint_to_posed() * extent));
    float y = dot(Jy, delta);
    Vec3 Jz = cross(joint_to_posed() * Vec3(0, 0, 1), current - (joint_to_posed() * extent));
    float z = dot(Jz, delta);
    angle_gradient = Vec3(x, y, z);
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2
    float T = 0.03f;

    // Do several iterations of Jacobian Transpose gradient descent for IK
    for (uint i = 0; i < 100; i++){
        Vec3 gradient = Vec3(0);
        for (uint handle = 0; handle < active_handles.size(); handle++){
            Joint *j = active_handles[handle]->joint;
            j->compute_gradient(active_handles[handle]->target, j->joint_to_posed() * j->extent);
            while(true){
                j->pose -= j->angle_gradient * T;
                if(j->is_root()){
                    break;
                }
                j = j->parent;
            }
        }
    }
}
