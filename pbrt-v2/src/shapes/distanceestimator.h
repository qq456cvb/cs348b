#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_DISTANCEESTIMATOR_H
#define PBRT_SHAPES_DISTANCEESTIMATOR_H

#include "shape.h"

struct DistanceEstimatorParams {
    int maxIters = 1000; // Number of steps along the ray until we give up (default 1000)
    float hitEpsilon = 1e-5; // how close to the surface we must be before we say we "hit" it
    float rayEpsilonMultiplier = 5; // how much we multiply hitEpsilon by to get rayEpsilon
    float normalEpsilon = 1e-5; // The epsilon we send to CalculateNormal()
};

class DistanceEstimator : public Shape {
public:
    DistanceEstimator(const Transform *o2w, const Transform *w2o, bool ro, const DistanceEstimatorParams& deParams);
    virtual Vector CalculateNormal(const Point& pos, float eps) const;
    virtual float Evaluate(const Point& p) const = 0;
    virtual bool Intersect(const Ray &ray, float *tHit,
                           float *rayEpsilon, DifferentialGeometry *dg) const;
    virtual bool IntersectP(const Ray &ray) const;
protected:
    DistanceEstimatorParams params;
};



#endif // PBRT_SHAPES_DISTANCEESTIMATOR_H