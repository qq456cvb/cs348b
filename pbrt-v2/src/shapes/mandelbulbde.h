//
//  mandelbulbde.h
//  pbrt
//
//  Created by Neil on 7/19/16.
//
//

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_SHAPES_MANDELBULBDE_H
#define PBRT_SHAPES_MANDELBULBDE_H

#include "distanceestimator.h"

class MandelBulbDE : public DistanceEstimator {
public:
    MandelBulbDE(const Transform *o2w, const Transform *w2o, bool ro, int fractalIters, int mandelbulbPower, const DistanceEstimatorParams& deParams);
    virtual float Evaluate(const Point& p) const;
    virtual BBox ObjectBound() const;
    virtual float Area() const;
private:
    int fractalIters;
    int mandelbulbPower;
};

MandelBulbDE *CreateMandelBulbDEShape(const Transform *o2w,
                                                      const Transform *w2o,
                                                      bool reverseOrientation, const ParamSet &params);

#endif /* mandelbulbde_h */
