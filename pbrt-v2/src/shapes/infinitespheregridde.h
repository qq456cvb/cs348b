#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_INFINITESPHEREGRIDDE_H
#define PBRT_SHAPES_INFINITESPHEREGRIDDE_H

#include "distanceestimator.h"

class InfiniteSphereGridDE : public DistanceEstimator {
public:
    InfiniteSphereGridDE(const Transform *o2w, const Transform *w2o, bool ro, float rad, float csz, const DistanceEstimatorParams& deParams);
    virtual float Evaluate(const Point& p) const;
    virtual BBox ObjectBound() const;
    virtual float Area() const;
private:
    float radius;
    float cellSize;
};

InfiniteSphereGridDE *CreateInfiniteSphereGridDEShape(const Transform *o2w,
                                                      const Transform *w2o,
                              bool reverseOrientation, const ParamSet &params);

#endif /* SphereDE_h */
