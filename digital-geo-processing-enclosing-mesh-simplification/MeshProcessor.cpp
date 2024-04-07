// MeshProcessor.cpp

#include "MeshProcessor.h"
#include <cmath>

void MeshProcessor::populateEdgeQueue(Mesh* mesh) {
    // Populate the edge queue with edges sorted by length
    for (size_t i = 0; i < mesh->edges.size(); ++i) {
        edgeQueue.push(mesh->edges[i]);
    }
}

void MeshProcessor::updateEdgeCosts(Mesh* mesh) {
    // Update the costs of all edges in the mesh
    for (size_t i = 0; i < mesh->edges.size(); ++i) {
        Edge* edge = mesh->edges[i];

        // Calculate edge length
        float edgeLength = calculateEdgeLength(edge, mesh);
        edge->length = edgeLength;
    }

    // Create a new priority queue based on the updated edge lengths
    std::priority_queue<Edge*, std::vector<Edge*>, EdgeComparator> updatedQueue;
    for (size_t i = 0; i < mesh->edges.size(); ++i) {
        updatedQueue.push(mesh->edges[i]);
    }

    // Update the edgeQueue member variable with the new queue
    edgeQueue = updatedQueue;
}


float MeshProcessor::calculateEdgeLength(Edge* edge, Mesh* mesh) {
    // Print the indices of the vertices for debugging
    std::cout << "Vertex indices: " << edge->v1i << ", " << edge->v2i << std::endl;

    // Get the vertices associated with the edge
    Vertex* v1 = mesh->verts[edge->v1i];
    Vertex* v2 = mesh->verts[edge->v2i];

    // Compute the Euclidean distance between the vertices
    float dx = v2->coords[0] - v1->coords[0];
    float dy = v2->coords[1] - v1->coords[1];
    float dz = v2->coords[2] - v1->coords[2];

    return sqrt(dx * dx + dy * dy + dz * dz); // Euclidean distance formula
}

void MeshProcessor::collapseEdge(Edge* edge, Mesh* mesh) {
    // Get the vertices associated with the edge
    Vertex* v1 = mesh->verts[edge->v1i];
    Vertex* v2 = mesh->verts[edge->v2i];

    // Compute the midpoint of the edge
    float midX = (v1->coords[0] + v2->coords[0]) / 2.0f;
    float midY = (v1->coords[1] + v2->coords[1]) / 2.0f;
    float midZ = (v1->coords[2] + v2->coords[2]) / 2.0f;

    // Update the coordinates of v1 to the midpoint
    v1->coords[0] = midX;
    v1->coords[1] = midY;
    v1->coords[2] = midZ;

    // Update vertex indices in triangles
    for (size_t i = 0; i < mesh->tris.size(); ++i) {
        Triangle* triangle = mesh->tris[i];
        if (triangle->v1i == edge->v2i) triangle->v1i = edge->v1i;
        if (triangle->v2i == edge->v2i) triangle->v2i = edge->v1i;
        if (triangle->v3i == edge->v2i) triangle->v3i = edge->v1i;
    }

    // Remove v2 from the vertex list
    auto v2Iterator = std::find(mesh->verts.begin(), mesh->verts.end(), v2);
    if (v2Iterator != mesh->verts.end()) {
        mesh->verts.erase(v2Iterator);
    }

    // Update neighboring vertices of v1 and their references to v2
    for (size_t i = 0; i < v2->vertList.size(); ++i) {
        int neighborVertexIndex = v2->vertList[i];
        Vertex* neighborVertex = mesh->verts[neighborVertexIndex];
        if (neighborVertex != v1) {
            for (size_t j = 0; j < neighborVertex->vertList.size(); ++j) {
                if (neighborVertex->vertList[j] == edge->v2i) {
                    neighborVertex->vertList[j] = edge->v1i;
                    break;
                }
            }
        }
    }

    // Update neighboring triangles of v1
    for (size_t i = 0; i < v2->triList.size(); ++i) {
        int tIndex = v2->triList[i];
        Triangle* neighborTriangle = mesh->tris[tIndex];
        if (neighborTriangle->v1i == edge->v1i || neighborTriangle->v2i == edge->v1i || neighborTriangle->v3i == edge->v1i) {
            // Remove the neighbor triangle from v1's triList
            v1->triList.push_back(tIndex);

            // Update the indices in the neighboring triangle's triList
            for (size_t j = 0; j < 3; ++j) {
                if (neighborTriangle->v1i == edge->v2i) neighborTriangle->v1i = edge->v1i;
                if (neighborTriangle->v2i == edge->v2i) neighborTriangle->v2i = edge->v1i;
                if (neighborTriangle->v3i == edge->v2i) neighborTriangle->v3i = edge->v1i;
            }

            // Remove v2 from the neighbor triangle's vertex indices
            auto triangleIterator = std::find(mesh->tris.begin(), mesh->tris.end(), neighborTriangle);
            if (triangleIterator != mesh->tris.end()) {
                mesh->tris.erase(triangleIterator);
                delete neighborTriangle; // Free memory for the removed triangle
            }
        }
    }

    // Update the costs of neighboring edges affected by the vertex collapse
    updateEdgeCosts(mesh);
}

void MeshProcessor::simplifyMesh(Mesh* mesh, int targetFaceCount) {
    // Populate the edge queue initially
    populateEdgeQueue(mesh);

    // Perform edge collapse operations until the desired face count is achieved
    while (!edgeQueue.empty() && mesh->tris.size() > targetFaceCount) {
        // Pop the top edge from the priority queue
        Edge* edge = edgeQueue.top();
        edgeQueue.pop();

        // Check if the edge can be collapsed
        if (canCollapseEdge(edge, mesh)) {
            // Collapse the edge only if it can be collapsed
            collapseEdge(edge, mesh);

            // Update edge costs after collapse
            updateEdgeCosts(mesh);
        }
        // If the edge cannot be collapsed, continue to the next edge
    }
}

bool MeshProcessor::canCollapseEdge(Edge* edge, Mesh* mesh) {
    // Check if the edge is a boundary edge
    if (isBoundaryEdge(edge, mesh)) {
        return false;
    }

    // Check if collapsing the edge would cause any vertex to become non-manifold
    if (wouldCauseNonManifold(edge, mesh)) {
        return false;
    }

    // Check if collapsing the edge would cause any triangle to become degenerate
    if (wouldCauseDegenerateTriangles(edge, mesh)) {
        return false;
    }

    // Create a Point object representing the midpoint of the edge
    Point midpoint;
    midpoint.coords[0] = (mesh->verts[edge->v1i]->coords[0] + mesh->verts[edge->v2i]->coords[0]) / 2.0;
    midpoint.coords[1] = (mesh->verts[edge->v1i]->coords[1] + mesh->verts[edge->v2i]->coords[1]) / 2.0;
    midpoint.coords[2] = (mesh->verts[edge->v1i]->coords[2] + mesh->verts[edge->v2i]->coords[2]) / 2.0;

    // Compute the winding number of the midpoint
    mesh->windingNumberByYusufSahillioglu(&midpoint); // Pass the address of the midpoint to the function

    // Check if the winding number indicates that the midpoint is inside the mesh
    if (midpoint.winding > 0.5) {
        // Midpoint is inside the mesh, edge can be collapsed
        return true;
    }
    else {
        // Midpoint is outside the mesh, edge cannot be collapsed
        return false;
    }
}

bool MeshProcessor::isBoundaryEdge(Edge* edge, Mesh* mesh) {
    // Get the vertices associated with the edge
    Vertex* v1 = mesh->verts[edge->v1i];
    Vertex* v2 = mesh->verts[edge->v2i];

    // Check if either vertex is not connected to any other vertex
    bool isBoundary = (v1->vertList.size() == 1) || (v2->vertList.size() == 1);

    return isBoundary;
}

bool MeshProcessor::wouldCauseNonManifold(Edge* edge, Mesh* mesh) {
    // Get the vertices associated with the edge
    Vertex* v1 = mesh->verts[edge->v1i];
    Vertex* v2 = mesh->verts[edge->v2i];

    // Count the number of triangles adjacent to v1 and v2
    int countV1 = 0, countV2 = 0;
    for (size_t i = 0; i < mesh->tris.size(); ++i) {
        Triangle* triangle = mesh->tris[i];
        // Check if the triangle contains v1 or v2
        if (triangle->v1i == edge->v1i || triangle->v2i == edge->v1i || triangle->v3i == edge->v1i) {
            countV1++;
        }
        if (triangle->v1i == edge->v2i || triangle->v2i == edge->v2i || triangle->v3i == edge->v2i) {
            countV2++;
        }
    }

    // Check if collapsing the edge would make v1 or v2 non-manifold
    return countV1 > 2 || countV2 > 2;
}

bool MeshProcessor::wouldCauseDegenerateTriangles(Edge* edge, Mesh* mesh) {
    // Get the vertices associated with the edge
    Vertex* v1 = mesh->verts[edge->v1i];
    Vertex* v2 = mesh->verts[edge->v2i];

    // Define a small threshold value for epsilon
    const float epsilon = 1e-6f; // You can adjust this value based on your requirements

    // Iterate through each triangle and check if it contains the edge
    for (size_t i = 0; i < mesh->tris.size(); ++i) {
        Triangle* triangle = mesh->tris[i];
        if (triangle->v1i == edge->v1i || triangle->v2i == edge->v1i || triangle->v3i == edge->v1i ||
            triangle->v1i == edge->v2i || triangle->v2i == edge->v2i || triangle->v3i == edge->v2i) {
            // Calculate the area of the triangle before and after collapsing the edge
            float areaBefore = calculateTriangleArea(mesh->verts[triangle->v1i], mesh->verts[triangle->v2i], mesh->verts[triangle->v3i]);
            float areaAfter = calculateTriangleArea(v1, v2, mesh->verts[triangle->v1i]) +
                calculateTriangleArea(v1, v2, mesh->verts[triangle->v2i]) +
                calculateTriangleArea(v1, v2, mesh->verts[triangle->v3i]);

            // Check if the area after collapsing is too small (almost degenerate)
            if (areaAfter < epsilon) {
                return true; // Triangle becomes degenerate
            }
        }
    }

    // No triangle becomes degenerate after collapsing the edge
    return false;
}

// Function to calculate the area of a triangle given its three vertices
float MeshProcessor::calculateTriangleArea(Vertex* v1, Vertex* v2, Vertex* v3) {
    // Calculate the vectors representing two edges of the triangle
    float edge1[3] = { v2->coords[0] - v1->coords[0], v2->coords[1] - v1->coords[1], v2->coords[2] - v1->coords[2] };
    float edge2[3] = { v3->coords[0] - v1->coords[0], v3->coords[1] - v1->coords[1], v3->coords[2] - v1->coords[2] };

    // Calculate the cross product of the two edges
    float crossProduct[3];
    crossProduct[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
    crossProduct[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
    crossProduct[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];

    // Calculate the magnitude of the cross product to get the area of the triangle
    float area = 0.5f * sqrt(crossProduct[0] * crossProduct[0] + crossProduct[1] * crossProduct[1] + crossProduct[2] * crossProduct[2]);

    return area;
}