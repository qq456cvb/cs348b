//
//  mandelbulbde.cpp
//  pbrt
//
//  Created by Neil on 7/19/16.
//
//
#include <stdio.h>
#include "stdafx.h"
#include "shapes/mandelbulbde.h"
#include "montecarlo.h"
#include "paramset.h"

MandelBulbDE::MandelBulbDE(const Transform *o2w, const Transform *w2o, bool ro, int fractalIters, int mandelbulbPower, const DistanceEstimatorParams& deParams)
: DistanceEstimator(o2w, w2o, ro, deParams)
{
    this->fractalIters = fractalIters;
    this->mandelbulbPower = mandelbulbPower;
}

BBox MandelBulbDE::ObjectBound() const {
    return BBox(Point(-1, -1, -1),
                Point( 1,  1,  1));
}

float MandelBulbDE::Evaluate(const Point &c) const {
    const float Power = (float)mandelbulbPower;
    Point z = c;
    float dr = 1.0;
    float r = 0.0;
    for (int i = 0; i < fractalIters; ++i) {
        r = (z-Point(0,0,0)).Length();
//        r = fabsf(z.x) + fabsf(z.y) + fabsf(z.z);
        // Escaped orbit
        if (r > 2.0f) {
            break;
        }
        
        // plus 1 to enlarge minimum distance
        dr =  pow( r, Power-1.0)*Power*dr + 1.0;
        
        // In original Mandelbrot this is z = z^2 + c
        // Mandelbulb does this by analogy, replacing the square with
        // scaling and rotating
        
        // create a 3-d real and imaginary space with (1, i, j) but no k part
        // Polar coordinates
        float theta = acos(z.z/r);
        float phi = atan2(z.y,z.x);
        
        // "square" z (really scaling and rotating)
        float zr = pow(r,Power);
        theta = theta*Power;
        phi = phi*Power;
        
        // Cartesian coordinates
        z = zr * Point(sin(theta)*cos(phi), sin(phi)*sin(theta), cos(theta));
        z += c;
    }
    //
    return 0.5 * (r/dr) * log(r);
}

float MandelBulbDE::Area() const {
    return std::numeric_limits<float>::infinity();
}

MandelBulbDE *CreateMandelBulbDEShape(const Transform *o2w, const Transform *w2o,
                                                      bool reverseOrientation, const ParamSet &params) {
    int fractalIters = params.FindOneInt("fractaliters", 1000);
    int mandelbulbPower = params.FindOneInt("mandelbulbpower", 8);
    DistanceEstimatorParams param;
    param.maxIters = params.FindOneInt("maxiters", 1000);
    param.hitEpsilon = params.FindOneFloat("hitEpsilon", 1e-6f);
    param.rayEpsilonMultiplier = params.FindOneFloat("rayEpsilonMultiplier", 1e2f);
    param.normalEpsilon = params.FindOneFloat("normalEpsilon", 1e-2f);
    return new MandelBulbDE(o2w, w2o, reverseOrientation, fractalIters, mandelbulbPower, param);
}