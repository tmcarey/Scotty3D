
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build_helper(size_t parentIdx,
                                  size_t max_leaf_size){
    //Iterate over each axis
    Node parent = nodes[parentIdx];
    if(parent.size <= max_leaf_size){
        return;
    }
    size_t partCount = 50;
    BBox b1Min;
    BBox b2Min;
    uint primCountMin1 = 0;
    uint primCountMin2 = 0;
    float minSahMetric = FLT_MAX;
    size_t minAxis = 0;
    float minPartVal = 0;
    for(size_t axis = 0; axis < 3; axis++){
        for(size_t partIdx = 0; partIdx < partCount; partIdx++){
            float gap = parent.bbox.max[axis] - parent.bbox.min[axis];
            float partVal = parent.bbox.min[axis] + (gap * ((float)partIdx / (float)partCount));
            auto sep = std::partition(primitives.begin() + parent.start, 
                                      primitives.begin() + parent.start + parent.size, 
                                      [axis, partVal](const auto& prim){ return prim.bbox().center()[axis] >= partVal; });
            BBox b1;
            uint primCount1 = 0;
            BBox b2;
            uint primCount2 = 0;
            for(auto it = primitives.begin() + parent.start; it < primitives.begin() + parent.start + parent.size; it++){
                if(it < sep){
                    b1.enclose(it->bbox());
                    primCount1++; 
                }else{
                    b2.enclose(it->bbox());
                    primCount2++;
                }
            }
            float sahMetric = (b1.surface_area() * primCount1 + b2.surface_area() * primCount2);
            if(sahMetric < minSahMetric){
                b1Min = b1;
                b2Min = b2;
                primCountMin1 = primCount1;
                primCountMin2 = primCount2;
                minSahMetric = sahMetric;
                minPartVal = partVal;
                minAxis = axis;
            }
        }
    }
    if(primCountMin1 == 0 || primCountMin2 == 0){
        nodes[parentIdx].l = 0;
        nodes[parentIdx].r = 0;
        return;
    }
    auto sep = std::partition(primitives.begin() + parent.start, primitives.begin() + parent.start + parent.size, [minAxis, minPartVal](const auto& prim){ return prim.bbox().center()[minAxis] >= minPartVal; });
    size_t lNodeIdx = new_node(b1Min, parent.start, 
                               std::distance(primitives.begin() + parent.start, sep), 0, 0);
    nodes[parentIdx].l = lNodeIdx;
    build_helper(lNodeIdx, max_leaf_size);
    size_t rNodeIdx = new_node(b2Min, std::distance(primitives.begin(), sep), std::distance(sep, primitives.begin() + parent.start + parent.size), 0, 0);
    nodes[parentIdx].r = rNodeIdx;
    build_helper(rNodeIdx, max_leaf_size);
}

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these
    BBox box;
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());

    size_t initNode = new_node(box, 0, primitives.size(), 0, 0);
    build_helper(initNode, max_leaf_size);
    root_idx = 0;
}

template<typename Primitive> void BVH<Primitive>::find_closest_hit(const Ray& ray, uint nodeIdx, Trace* closest) const {
    Node node = nodes[nodeIdx];
    if(node.l == 0 && node.r == 0){
        Trace ret = *closest;
        for(auto it = (primitives.begin() + node.start); it < (primitives.begin() + node.start + node.size); it++) {
            Trace hit = it->hit(ray);
            ret = Trace::min(ret, hit);
        }
        *closest = ret;
    }else{
        Vec2 hit1; 
        Vec2 hit2;
        bool didHit1 = nodes[node.l].bbox.hit(ray, hit1);
        bool didHit2 = nodes[node.r].bbox.hit(ray, hit2);

        uint firstNode = node.l;
        uint secondNode = node.r;
        bool hitFirst = didHit1;
        bool hitSecond = didHit2;
        Vec2 secondHitVec = hit2;
        if(hit1.x >= hit2.x){
            firstNode = node.r;
            secondNode = node.l;
            hitFirst = didHit2;
            hitSecond = didHit1;
            secondHitVec = hit1;
        }


        if(hitFirst){
            find_closest_hit(ray, firstNode, closest);
        }
        if(hitSecond && (!closest->hit || closest->distance >= secondHitVec.x)){
            find_closest_hit(ray, secondNode, closest);
        }
    }

}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace t;
    find_closest_hit(ray, 0, &t);
    return t;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c + lvl, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
