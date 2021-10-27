
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    auto h = v->halfedge();
    bool twinFlag = true;
    do {
        if(twinFlag){
            h = h->twin();
        }else{
            h = h->next();
        }
        twinFlag = !twinFlag;
    } while(h != v->halfedge());
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {
    Halfedge_Mesh::HalfedgeRef mainEdge = e->_halfedge;
    Halfedge_Mesh::HalfedgeRef mainNext = mainEdge->next();
    Halfedge_Mesh::HalfedgeRef mainPrev = mainEdge;
    while(mainPrev->next() != mainEdge) {
        mainPrev = mainPrev->next();
    }

    Halfedge_Mesh::HalfedgeRef twinEdge = mainEdge->twin();
    Halfedge_Mesh::HalfedgeRef twinNext = twinEdge->next();
    Halfedge_Mesh::HalfedgeRef twinPrev = twinEdge;
    while(twinPrev->next() != twinEdge) {
        twinPrev = twinPrev->next();
    }

    auto mainFace = mainEdge->face();
    twinPrev->_next = mainNext;
    mainPrev->_next = twinNext;
    mainEdge->vertex()->_halfedge = twinNext;
    twinEdge->vertex()->_halfedge = mainNext;
    if(mainEdge == mainEdge->twin()->next()){
        erase(mainEdge->vertex());
    }
    if(twinEdge == twinEdge->twin()->next()){
        erase(twinEdge->vertex());
    }
    erase(mainEdge->_edge);
    erase(mainEdge);
    erase(twinEdge);
    mainFace->_halfedge = mainNext;
    if(mainFace != twinEdge->face()){
        erase(twinEdge->face());
    }

    auto it = mainNext;
    do{
        it->_face = mainFace;
        it = it->next();
    }while(it != mainNext);

    return mainFace;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    Halfedge_Mesh::HalfedgeRef mainEdge = e->halfedge();
    Halfedge_Mesh::HalfedgeRef mainEdgeNext = mainEdge->next();
    Halfedge_Mesh::HalfedgeRef mainPrevEdge = mainEdge;
    Halfedge_Mesh::FaceRef mainFace = mainEdge->face();
    while(mainPrevEdge->next() != mainEdge) {
        mainPrevEdge = mainPrevEdge->next();
    }

    Halfedge_Mesh::HalfedgeRef twinEdge = mainEdge->twin();
    Halfedge_Mesh::HalfedgeRef twinEdgeNext = twinEdge->next();
    Halfedge_Mesh::HalfedgeRef twinPrevEdge = twinEdge;
    Halfedge_Mesh::FaceRef twinFace = twinEdge->face();
    while(twinPrevEdge->next() != twinEdge) {
        twinPrevEdge = twinPrevEdge->next();
    }

    Halfedge_Mesh::VertexRef toErase = mainEdge->vertex();
    Halfedge_Mesh::VertexRef replacer = twinEdge->vertex();
    Halfedge_Mesh::HalfedgeRef it = mainEdge;
    do {
        it->_vertex = replacer;
        it = it->twin()->next();
    } while(it != mainEdge);

    mainPrevEdge->_next = mainEdgeNext;
    mainFace->_halfedge = mainEdgeNext;
    if(mainPrevEdge->next() == mainEdgeNext) {
        mainPrevEdge->twin()->_twin = mainEdgeNext->twin();
        mainPrevEdge->vertex()->_halfedge = mainEdgeNext->twin();
        mainEdgeNext->twin()->_twin = mainPrevEdge->twin();
        mainPrevEdge->edge()->_halfedge = mainPrevEdge->twin();
        mainEdgeNext->twin()->_edge = mainPrevEdge->edge();
        erase(mainEdgeNext->edge());
        erase(mainPrevEdge);
        erase(mainEdgeNext);
        erase(mainFace);
    }
    twinPrevEdge->_next = twinEdgeNext;
    twinFace->_halfedge = twinEdgeNext;
    if(twinPrevEdge->next() == twinEdgeNext) {
        twinPrevEdge->twin()->_twin = twinEdgeNext->twin();
        twinPrevEdge->vertex()->_halfedge = twinEdgeNext->twin();
        twinEdgeNext->twin()->_twin = twinPrevEdge->twin();
        twinPrevEdge->edge()->_halfedge = twinPrevEdge->twin();
        twinEdgeNext->twin()->_edge = twinPrevEdge->edge();
        erase(twinEdgeNext->edge());
        erase(twinPrevEdge);
        erase(twinEdgeNext);
        erase(twinFace);
    }

    replacer->pos = (replacer->pos + toErase->pos) * 0.5f;
    replacer->_halfedge = twinPrevEdge->twin();

    erase(mainEdge);
    erase(twinEdge);
    erase(toErase);
    erase(e);
    return replacer;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    Halfedge_Mesh::HalfedgeRef mainEdge = e->halfedge();
    Halfedge_Mesh::HalfedgeRef mainNextEdge = mainEdge->next();
    Halfedge_Mesh::HalfedgeRef mainPrevEdge = mainEdge;
    Halfedge_Mesh::HalfedgeRef mainPrevPrevEdge = mainEdge;
    Halfedge_Mesh::FaceRef mainFace = mainEdge->face();
    while(mainPrevEdge->next() != mainEdge) {
        mainPrevPrevEdge = mainPrevEdge;
        mainPrevEdge = mainPrevEdge->next();
    }
    Halfedge_Mesh::VertexRef mainVertex = mainEdge->vertex();
    Halfedge_Mesh::HalfedgeRef twinEdge = mainEdge->twin();
    Halfedge_Mesh::HalfedgeRef twinNextEdge = twinEdge->next();
    Halfedge_Mesh::HalfedgeRef twinPrevEdge = twinEdge;
    Halfedge_Mesh::HalfedgeRef twinPrevPrevEdge = twinEdge;
    Halfedge_Mesh::FaceRef twinFace = twinEdge->face();
    while(twinPrevEdge->next() != twinEdge) {
        twinPrevPrevEdge = twinPrevEdge;
        twinPrevEdge = twinPrevEdge->next();
    }

    Halfedge_Mesh::VertexRef twinVertex = twinEdge->vertex();

    mainVertex->_halfedge = twinNextEdge;
    mainFace->_halfedge = mainEdge;

    mainPrevPrevEdge->set_neighbors(mainEdge, mainPrevPrevEdge->twin(), mainPrevPrevEdge->vertex(),
                                    mainPrevPrevEdge->edge(), mainPrevPrevEdge->face());
    mainPrevEdge->set_neighbors(twinNextEdge, mainPrevEdge->twin(), mainPrevEdge->vertex(),
                                mainPrevEdge->edge(), twinFace);
    mainEdge->set_neighbors(twinPrevEdge, twinEdge, mainPrevEdge->vertex(), mainEdge->edge(),
                            mainEdge->face());

    twinVertex->_halfedge = mainNextEdge;
    twinFace->_halfedge = twinEdge;

    twinPrevPrevEdge->set_neighbors(twinEdge, twinPrevPrevEdge->twin(), twinPrevPrevEdge->vertex(),
                                    twinPrevPrevEdge->edge(), twinPrevPrevEdge->face());
    twinPrevEdge->set_neighbors(mainNextEdge, twinPrevEdge->twin(), twinPrevEdge->vertex(),
                                twinPrevEdge->edge(), mainFace);
    twinEdge->set_neighbors(mainPrevEdge, mainEdge, twinPrevEdge->vertex(), twinEdge->edge(),
                            twinEdge->face());

    return e;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    Halfedge_Mesh::VertexRef newVertex = new_vertex();

    Halfedge_Mesh::HalfedgeRef mainEdge = e->_halfedge;
    Halfedge_Mesh::VertexRef mainVertex = mainEdge->vertex();
    Halfedge_Mesh::HalfedgeRef mainNextEdge = mainEdge->next();
    Halfedge_Mesh::HalfedgeRef mainPrevEdge = mainNextEdge->next();
    Halfedge_Mesh::HalfedgeRef twinEdge = mainEdge->twin();
    Halfedge_Mesh::HalfedgeRef twinNextEdge = twinEdge->next();
    Halfedge_Mesh::HalfedgeRef twinPrevEdge = twinNextEdge->next();
    Halfedge_Mesh::VertexRef twinVertex = twinEdge->vertex();

    Halfedge_Mesh::EdgeRef newCoreEdge = new_edge();
    Halfedge_Mesh::HalfedgeRef newCoreEdgeMain = new_halfedge();
    newCoreEdge->_halfedge = newCoreEdgeMain;
    Halfedge_Mesh::HalfedgeRef newCoreEdgeTwin = new_halfedge();

    Halfedge_Mesh::EdgeRef mainFaceEdge = new_edge();
    Halfedge_Mesh::HalfedgeRef mainFaceEdgeMain = new_halfedge();
    mainFaceEdge->_halfedge = mainFaceEdgeMain;
    Halfedge_Mesh::HalfedgeRef mainFaceEdgeTwin = new_halfedge();
    Halfedge_Mesh::FaceRef mainFace = mainEdge->face();
    Halfedge_Mesh::FaceRef newMainFace = new_face();
    newMainFace->_halfedge = mainFaceEdgeTwin;

    Halfedge_Mesh::EdgeRef twinFaceEdge = new_edge();
    Halfedge_Mesh::HalfedgeRef twinFaceEdgeMain = new_halfedge();
    twinFaceEdge->_halfedge = twinFaceEdgeMain;
    Halfedge_Mesh::HalfedgeRef twinFaceEdgeTwin = new_halfedge();
    Halfedge_Mesh::FaceRef twinFace = twinEdge->face();
    Halfedge_Mesh::FaceRef newTwinFace = new_face();
    newTwinFace->_halfedge = twinFaceEdgeTwin;

    mainFace->_halfedge = mainEdge;
    twinFace->_halfedge = twinEdge;
    newVertex->pos = (mainVertex->pos + twinVertex->pos) / 2.0f;
    newVertex->_halfedge = mainEdge;
    mainVertex->_halfedge = twinNextEdge;
    mainEdge->_vertex = newVertex;

    mainNextEdge->set_neighbors(mainFaceEdgeMain, mainNextEdge->twin(), mainNextEdge->vertex(),
                                mainNextEdge->edge(), mainFace);
    mainPrevEdge->set_neighbors(newCoreEdgeMain, mainPrevEdge->twin(), mainPrevEdge->vertex(),
                                mainPrevEdge->edge(), newMainFace);
    twinEdge->set_neighbors(twinFaceEdgeMain, mainEdge, twinEdge->vertex(), twinEdge->edge(),
                            twinFace);
    twinNextEdge->set_neighbors(twinFaceEdgeTwin, twinNextEdge->twin(), twinNextEdge->vertex(),
                                twinNextEdge->edge(), newTwinFace);

    newCoreEdgeMain->set_neighbors(mainFaceEdgeTwin, newCoreEdgeTwin, mainVertex, newCoreEdge,
                                   newMainFace);
    newCoreEdgeTwin->set_neighbors(twinNextEdge, newCoreEdgeMain, newVertex, newCoreEdge,
                                   newTwinFace);

    mainFaceEdgeMain->set_neighbors(mainEdge, mainFaceEdgeTwin, mainPrevEdge->vertex(),
                                    mainFaceEdge, mainFace);
    mainFaceEdgeTwin->set_neighbors(mainPrevEdge, mainFaceEdgeMain, newVertex, mainFaceEdge,
                                    newMainFace);

    twinFaceEdgeMain->set_neighbors(twinPrevEdge, twinFaceEdgeTwin, newVertex, twinFaceEdge,
                                    twinFace);
    twinFaceEdgeTwin->set_neighbors(newCoreEdgeTwin, twinFaceEdgeMain, twinPrevEdge->vertex(),
                                    twinFaceEdge, newTwinFace);

    return newVertex;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these
    positions, as well as the normal and tangent offset fields to assign positions to
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    auto h = v->halfedge();
    int newEdgeCount = 0;
    do {
        h = h->twin();
        h = h->next();
        newEdgeCount++;
    } while(h != v->halfedge());

    if(newEdgeCount < 3){
      return std::nullopt;
    }

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."
    h = v->halfedge();
    auto newFace = new_face();
    auto newHEdge = new_halfedge();
    auto newHEdgeTwin = new_halfedge();
    auto newVertex = new_vertex();
    auto newEdge = new_edge();
    newFace->_halfedge = newHEdgeTwin;
    auto firstEdgeTwin = newHEdgeTwin;
    for(int i = 1; i < newEdgeCount; i++){
    }
    auto prev = h;
    while(prev->next() != h){
        prev = prev->next();
    }
    newEdge->_halfedge = newHEdge;
    prev->set_neighbors (newHEdge, prev->twin(), prev->vertex(), prev->edge(), prev->face());
    newHEdge->set_neighbors(h, newHEdgeTwin, newVertex, newEdge, h->face());
    newHEdgeTwin->set_neighbors(h, newHEdge, newVertex, newEdge, newFace);
    auto lastEdgeTwin = newHEdgeTwin;
    auto lastOriginalTwin = h->twin();
    h = h->twin();
    h = h->next();
    while(h != v->halfedge()){
        newHEdge = new_halfedge();
        newHEdgeTwin = new_halfedge();
        newVertex = new_vertex();
        newEdge = new_edge();
        newEdge->_halfedge = newHEdge;
        lastOriginalTwin->set_neighbors(newHEdge, lastOriginalTwin->twin(), lastOriginalTwin->vertex(), lastEdgeTwin->edge(), lastEdgeTwin->face());
        prev = h;
        while(prev->next() != h){
            prev = prev->next();
        }
        h->set_neighbors(h->next(), h->twin(), newVertex, h->edge(), h->face());
        prev->set_neighbors (newHEdge, prev->twin(), prev->vertex(), prev->edge(), prev->face());
        newHEdge->set_neighbors(h, newHEdgeTwin, newVertex, newEdge, newFace);
        newHEdgeTwin->set_neighbors(lastEdgeTwin, newHEdge, newVertex, newEdge, newFace);
        lastEdgeTwin = newHEdgeTwin;
        h = h->twin();
        h = h->next();
    } 
    firstEdgeTwin->set_neighbors(lastEdgeTwin, firstEdgeTwin->twin(), firstEdgeTwin->vertex(), firstEdgeTwin->edge(), firstEdgeTwin->face());


    erase(v);
    return newFace;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."
    int faceLength = 1;
    Halfedge_Mesh::HalfedgeRef it = f->_halfedge;
    while(it->next() != f->halfedge()) {
        faceLength++;
        it = it->next();
    }

    Halfedge_Mesh::HalfedgeRef* orgEdges = new Halfedge_Mesh::HalfedgeRef[faceLength];
    Halfedge_Mesh::EdgeRef* newEdges = new Halfedge_Mesh::EdgeRef[faceLength];
    Halfedge_Mesh::HalfedgeRef* newEdgeMains = new Halfedge_Mesh::HalfedgeRef[faceLength];
    Halfedge_Mesh::HalfedgeRef* newEdgeTwins = new Halfedge_Mesh::HalfedgeRef[faceLength];
    Halfedge_Mesh::EdgeRef* newInwardEdges = new Halfedge_Mesh::EdgeRef[faceLength];
    Halfedge_Mesh::HalfedgeRef* newInwardEdgeMains = new Halfedge_Mesh::HalfedgeRef[faceLength];
    Halfedge_Mesh::HalfedgeRef* newInwardEdgeTwins = new Halfedge_Mesh::HalfedgeRef[faceLength];
    Halfedge_Mesh::VertexRef* orgVertices = new Halfedge_Mesh::VertexRef[faceLength];
    Halfedge_Mesh::VertexRef* newVertices = new Halfedge_Mesh::VertexRef[faceLength];
    Halfedge_Mesh::FaceRef* newFaces = new Halfedge_Mesh::FaceRef[faceLength];
    Halfedge_Mesh::FaceRef newFace = f;

    it = f->_halfedge;
    for(int i = 0; i < faceLength; i++) {
        orgEdges[i] = it;
        newEdges[i] = new_edge();
        newEdgeMains[i] = new_halfedge();
        newEdgeTwins[i] = new_halfedge();
        newInwardEdges[i] = new_edge();
        newInwardEdgeMains[i] = new_halfedge();
        newInwardEdgeTwins[i] = new_halfedge();
        orgVertices[i] = it->_vertex;
        newVertices[i] = new_vertex();
        newFaces[i] = new_face();
        it = it->next();
    }

    for(int i = 0; i < faceLength; i++) {
        int nextIdx = (((i + 1) % faceLength) + faceLength) % faceLength;
        int prevIdx = (((i - 1) % faceLength) + faceLength) % faceLength;
        orgEdges[i]->set_neighbors(newInwardEdgeTwins[nextIdx], orgEdges[i]->twin(),
                                   orgEdges[i]->vertex(), orgEdges[i]->edge(), newFaces[i]);
        newEdges[i]->_halfedge = newEdgeMains[i];
        newEdgeMains[i]->set_neighbors(newInwardEdgeMains[i], newEdgeTwins[i], newVertices[nextIdx],
                                       newEdges[i], newFaces[i]);
        newEdgeTwins[i]->set_neighbors(newEdgeTwins[nextIdx], newEdgeMains[i], newVertices[i],
                                       newEdges[i], newFace);
        newInwardEdges[i]->_halfedge = newInwardEdgeMains[i];
        newInwardEdgeMains[i]->set_neighbors(orgEdges[i], newInwardEdgeTwins[i], newVertices[i],
                                             newInwardEdges[i], newFaces[i]);
        newInwardEdgeTwins[i]->set_neighbors(newEdgeMains[prevIdx], newInwardEdgeMains[i],
                                             orgVertices[i], newInwardEdges[i], newFaces[prevIdx]);
        orgVertices[i]->_halfedge = orgEdges[i];
        newVertices[i]->_halfedge = newInwardEdgeMains[i];
        newVertices[i]->pos = orgVertices[i]->pos;
        newFaces[i]->_halfedge = orgEdges[i];
    }
    newFace->_halfedge = newEdgeTwins[0];

    return newFace;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    Vec3 avg = Vec3(0, 0, 0);
    Vec3 n = Vec3(0, 0, 0);
    size_t posCount = start_positions.size();
    for (uint i = 0; i < posCount; i++){
        Vec3 pi = start_positions[i];
        Vec3 pj = start_positions[(i + 1) % posCount];
        n += cross(pi, pj);
        avg += start_positions[i];
    }
    avg /= start_positions.size();
    n = n.unit();

    auto it = h;
    uint i = 0; 
    do{
        Vec3 tangent = (avg - start_positions[i]);
        it->vertex()->pos = start_positions[i] + (n * normal_offset * -1.) + (tangent_offset * tangent);
        it = it->next();
        i++;
    } while(it != h);

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
}

void Halfedge_Mesh::triangulateFace(Halfedge_Mesh::FaceRef face){
    auto h = face->halfedge();
    auto startEdge = face->halfedge();
    int i = 1;
    do{
        if(i >= 3 && startEdge != h->next()){
            auto splitEdge = h;
            auto prevStartEdge = startEdge;
            while(prevStartEdge->next() != startEdge) {
                prevStartEdge = prevStartEdge->next();
            }
            auto prevSplitEdge = startEdge->next();
            auto splitFace = new_face();
            auto splitFullEdge = new_edge();
            auto splitHEdgeMain = new_halfedge();
            auto splitHEdgeTwin = new_halfedge();

            prevStartEdge->set_neighbors(splitHEdgeMain, prevStartEdge->twin(), prevStartEdge->vertex(), prevStartEdge->edge(), splitFace);
            prevSplitEdge->_next = splitHEdgeTwin;
            splitFace->_halfedge = splitHEdgeMain;
            splitFullEdge->_halfedge = splitHEdgeMain;
            splitHEdgeMain->set_neighbors(splitEdge, splitHEdgeTwin, startEdge->vertex(), splitFullEdge, splitFace);
            splitHEdgeTwin->set_neighbors(startEdge, splitHEdgeMain, splitEdge->vertex(), splitFullEdge, face);
            auto it = splitEdge;
            do{
                it->_face = splitFace;
                it = it->next();
            }while(it != splitEdge);

            return;
        }
        h = h->next();
        i++;
    }while(startEdge != h);
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for(auto it = faces_begin(); it != faces_end(); it++){
        triangulateFace(it);
    }

}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for(auto it = vertices_begin(); it != vertices_end(); it++){
        it->new_pos = it->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for(auto it = edges_begin(); it != edges_end(); it++){
        Vec3 total = Vec3(0, 0, 0);
        total += it->halfedge()->vertex()->pos;
        total += it->halfedge()->twin()->vertex()->pos;
        it->new_pos = total / 2;
    }


    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for(auto it = faces_begin(); it != faces_end(); it++){
        Vec3 total = Vec3(0, 0, 0);
        auto h_it = it->_halfedge;
        int i = 0;
        do{
            total += h_it->vertex()->pos;
            h_it = h_it->next();
            i++;
        }while(h_it != it->_halfedge);
        it->new_pos = total / i;
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for(auto it = faces_begin(); it != faces_end(); it++){
        Vec3 total = Vec3(0, 0, 0);
        auto h_it = it->_halfedge;
        int i = 0;
        do{
            total += h_it->vertex()->pos;
            h_it = h_it->next();
            i++;
        }while(h_it != it->_halfedge);
        it->new_pos = total / i;
    }

    // Edges
    for(auto it = edges_begin(); it != edges_end(); it++){
        Vec3 total = Vec3(0, 0, 0);
        total += it->halfedge()->face()->new_pos;
        total += it->halfedge()->twin()->face()->new_pos;
        total += it->halfedge()->vertex()->pos;
        total += it->halfedge()->twin()->vertex()->pos;
        it->new_pos = total / 4;
    }

    // Vertices
    for(auto it = vertices_begin(); it != vertices_end(); it++){
        Vec3 q;
        Vec3 r;
        Vec3 s = it->pos;
        auto h_it = it->halfedge();
        uint count = 0;
        do{
            Vec3 edgeSum = it->pos + h_it->twin()->vertex()->pos;
            r += edgeSum / 2;
            q += h_it->face()->new_pos;
            h_it = h_it->twin()->next();
            count++;
        }while(h_it != it->halfedge());
        r /= count;
        q /= count;

        it->new_pos = (q + (2 * r) + (count - 3) * s) / count;
    }

}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.

    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.

    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new.
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)

    // Now flip any new edge that connects an old and new vertex.

    // Finally, copy new vertex positions into the Vertex::pos.
}

int Halfedge_Mesh::getDegree(VertexRef v){
    int d = 0;
    auto h = v->halfedge();
    do{
        d++;
        h = h->twin()->next();
    }while(h != v->halfedge());
    return d;
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    for(auto it = vertices_begin(); it != vertices_end(); it++){
        if(getDegree(it) != 3){
            return false;
        }
    }
    
    float L = 0;
    uint edgeCount = 0;
    for(auto it = edges_begin(); it != edges_end(); it++){
        Vec3 mid = it->halfedge()->vertex()->pos;
        mid -= it->halfedge()->twin()->vertex()->pos;
        L += mid.norm();
        edgeCount++;
    }
    L /= edgeCount;
    float minLength = 4 * L / 3;
    float maxLength = 4 * L / 5;

    std::vector<Halfedge_Mesh::EdgeRef> toSplit;
    std::vector<Halfedge_Mesh::EdgeRef> toCollapse;
    for(auto it = edges_begin(); it != edges_end(); it++){
        Vec3 mid = it->halfedge()->vertex()->pos;
        mid -= it->halfedge()->twin()->vertex()->pos;
        float length = mid.norm();
        if(length < minLength){
            toCollapse.push_back(it);
        }else if(length > maxLength){
            toSplit.push_back(it);
        }
    }

    for(uint i = 0; i < toCollapse.size(); i++){
        collapse_edge_erase(toCollapse[i]);
    }
    for(uint i = 0; i < toSplit.size(); i++){
        split_edge(toSplit[i]);
    }

    for(auto it = edges_begin(); it != edges_end(); it++){
        VertexRef a1 = it->halfedge()->vertex();
        VertexRef a2 = it->halfedge()->twin()->vertex();
        VertexRef b1 = it->halfedge()->next()->next()->vertex();
        VertexRef b2 = it->halfedge()->twin()->next()->next()->vertex();
        int initDev = abs(getDegree(a1) - 6)
                    + abs(getDegree(a2) - 6)
                    + abs(getDegree(b1) - 6)
                    + abs(getDegree(b2) - 6);
        int splitDev = abs((getDegree(a1) - 1) - 6)
                    + abs((getDegree(a2) - 1) - 6)
                    + abs((getDegree(b1) + 1) - 6)
                    + abs((getDegree(b2) + 1) - 6);
        if(splitDev < initDev){
            split_edge(it);
        }
    }

    float w = 0.2f;

    for(auto it = vertices_begin(); it != vertices_end(); it++){
        Vec3 centroid = it->neighborhood_center();
        Vec3 normal = it->normal();
        Vec3 toCentroid = centroid - it->pos;
        Vec3 toCentroidSansNorm = toCentroid - (normal * dot(toCentroid, normal));
        it->new_pos = it->pos + w * (toCentroidSansNorm + it->pos);
    }

    for(auto it = vertices_begin(); it != vertices_end(); it++){
        it->pos = it->new_pos;
    }

    return true;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
