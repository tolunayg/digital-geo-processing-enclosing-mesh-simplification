// MeshProcessor.h

#pragma once

#include "Mesh.h"
#include <queue>

struct EdgeComparator {
    bool operator()(const Edge* e1, const Edge* e2) const {
        return e1->length > e2->length; // Assuming shorter edges have higher priority
    }
};



class MeshProcessor {
private:
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeComparator> edgeQueue;

    void updateEdgeCosts(Mesh* mesh);
    void collapseEdge(Edge* edge, Mesh* mesh);
    float calculateEdgeLength(Edge* edge, Mesh* mesh);
    bool canCollapseEdge(Edge* edge, Mesh* mesh);
    bool isBoundaryEdge(Edge* edge, Mesh* mesh);
    bool wouldCauseNonManifold(Edge* edge, Mesh* mesh);
    bool wouldCauseDegenerateTriangles(Edge* edge, Mesh* mesh);
    float calculateTriangleArea(Vertex* v1, Vertex* v2, Vertex* v3);

public:
    void populateEdgeQueue(Mesh* mesh);
    void simplifyMesh(Mesh* mesh, int targetFaceCount);
};