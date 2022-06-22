#pragma once

#include "nanoflann/nanoflann.hpp"
#include "nanoflann/KDTreeVectorOfVectorsAdaptor.h"

#include "geometrycentral/surface/geometry.h"

#include <vector>

// Stupid nanoflann wrapper
typedef KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<double>>, double> KdTree;

namespace geometrycentral {
namespace surface {

struct SymmetryResult {
  bool symmetryFound = false;                  // was a symmetry found? is this data valid?
  std::vector<Vertex> canonicalVertices;       // a representative entry from each
                                               // set of symmetry pairs
  VertexData<std::vector<Vertex>> symmetrySet; // for each unique vertex,
                                               // all others vertices that
                                               // are symmetry pairs
};

KdTree* buildKDTree(SurfaceMesh& mesh, VertexPositionGeometry& geom);
bool findPoint(KdTree* tree, Vector3 target, double toleranceRadius, size_t& result);

// Look for a symmetry about a mirror plane
SymmetryResult detectSymmetryMirror(SurfaceMesh& mesh, VertexPositionGeometry& geom, Vector3 planeNormal, Vector3 planePoint);

SymmetryResult detectSymmetryMirror(SurfaceMesh& mesh, VertexPositionGeometry& geom, Vector3 planeNormal, Vector3 planePoint, KdTree* tree);

SymmetryResult detectSymmetryDoubleMirror(SurfaceMesh& mesh, VertexPositionGeometry& geom, KdTree* tree);

// Look for symmetry which is mirrored over the y and z planes
SymmetryResult detectSymmetryDoubleMirror(SurfaceMesh& mesh, VertexPositionGeometry& geom);

// Look for a rotational symmetry
SymmetryResult detectSymmetryRotation(SurfaceMesh& mesh, VertexPositionGeometry& geom, Vector3 rotAxis, Vector3 rotPoint, int nSym);

// Automatically search for the typical mirror and rotation symmetries about the
// shape center
// Returns any symmetry which is found.
SymmetryResult detectSymmetryAuto(SurfaceMesh& mesh, VertexPositionGeometry& geom);
SymmetryResult detectSymmetryAutoRotation(SurfaceMesh& mesh, VertexPositionGeometry& geom);

// Look for a symmetry about a mirror plane
SymmetryResult detectSymmetryAutoMirror(SurfaceMesh& mesh, VertexPositionGeometry& geom); 

} // namespace surface
} // namespace geometrycentral
