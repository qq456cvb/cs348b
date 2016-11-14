#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_SPHEREDE_H
#define PBRT_SHAPES_SPHEREDE_H

#include "distanceestimator.h"

class SphereDE : public DistanceEstimator {
public:
    SphereDE(const Transform *o2w, const Transform *w2o, bool ro, float rad, const DistanceEstimatorParams& deParams);
    virtual float Evaluate(const Point& p) const;
    virtual BBox ObjectBound() const;
    virtual float Area() const;
private:
    float radius;
};

SphereDE *CreateSphereDEShape(const Transform *o2w, const Transform *w2o,
        bool reverseOrientation, const ParamSet &params);

#endif /* SphereDE_h */
