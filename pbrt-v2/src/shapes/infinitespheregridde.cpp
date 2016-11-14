//
//  SphereDE.cpp
//  pbrt
//
//  Created by Neil on 7/19/16.
//
//

#include <stdio.h>
#include "stdafx.h"
#include "shapes/infinitespheregridde.h"
#include "montecarlo.h"
#include "paramset.h"

InfiniteSphereGridDE::InfiniteSphereGridDE(const Transform *o2w, const Transform *w2o, bool ro, float rad, float csz, const DistanceEstimatorParams& deParams)
: DistanceEstimator(o2w, w2o, ro, deParams)
{
    radius = rad;
    cellSize = csz;
}

BBox InfiniteSphereGridDE::ObjectBound() const {
    // PBRT cannot handle infinite numbers or max/min values for bounding box!!!
//    return BBox(Point(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min()),
//                Point(std::numeric_limits<float>::max(),  std::numeric_limits<float>::max(), std::numeric_limits<float>::max()));
    return BBox(Point(-1e5, -1e5, -1e5),
                Point( 1e5,  1e5,  1e5));
}

float InfiniteSphereGridDE::Evaluate(const Point &p) const {
    Point gridP = Point(remainderf(p.x, cellSize),
                        remainderf(p.y, cellSize),
                        remainderf(p.z, cellSize));
    return sqrtf(gridP.x*gridP.x+gridP.y*gridP.y+gridP.z*gridP.z) - radius;
}

float InfiniteSphereGridDE::Area() const {
    return std::numeric_limits<float>::infinity();
}

InfiniteSphereGridDE *CreateInfiniteSphereGridDEShape(const Transform *o2w, const Transform *w2o,
                              bool reverseOrientation, const ParamSet &params) {
    float radius = params.FindOneFloat("radius", 1.f);
    float cellSize = params.FindOneFloat("cellSize", 10.f);
    DistanceEstimatorParams param;
    param.maxIters = params.FindOneInt("maxiters", 1000);
    param.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-4f);
    param.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 1e2f);
    param.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-2f);
    return new InfiniteSphereGridDE(o2w, w2o, reverseOrientation, radius, cellSize, param);
}

