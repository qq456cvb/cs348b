//
//  SphereDE.cpp
//  pbrt
//
//  Created by Neil on 7/19/16.
//
//

#include <stdio.h>
#include "stdafx.h"
#include "shapes/spherede.h"
#include "montecarlo.h"
#include "paramset.h"

SphereDE::SphereDE(const Transform *o2w, const Transform *w2o, bool ro, float rad, const DistanceEstimatorParams& deParams)
    : DistanceEstimator(o2w, w2o, ro, deParams)
{
    radius = rad;
}

BBox SphereDE::ObjectBound() const {
    return BBox(Point(-radius, -radius, -radius),
                Point( radius,  radius, radius));
}

float SphereDE::Evaluate(const Point &p) const {
    return sqrtf(p.x*p.x+p.y*p.y+p.z*p.z) - radius;
}

float SphereDE::Area() const {
    return 4 * M_PI * radius * radius;
}

SphereDE *CreateSphereDEShape(const Transform *o2w, const Transform *w2o,
                                                bool reverseOrientation, const ParamSet &params) {
    float radius = params.FindOneFloat("radius", 1.f);
    DistanceEstimatorParams param;
    param.maxIters = params.FindOneInt("maxiters", 1000);
    param.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-6f);
    param.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 1e2f);
    param.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-5f);
    return new SphereDE(o2w, w2o, reverseOrientation, radius, param);
}

