#include "stdafx.h"
#include "shapes/distanceestimator.h"
#include "montecarlo.h"
#include "paramset.h"

DistanceEstimator::DistanceEstimator(const Transform *o2w, const Transform *w2o, bool ro,
               const DistanceEstimatorParams& deParams)
    : Shape(o2w, w2o, ro) {
    params = deParams;
}

Vector DistanceEstimator::CalculateNormal(const Point& pos, float eps) const {
    const Vector v1 = Vector( 1.0,-1.0,-1.0);
    const Vector v2 = Vector(-1.0,-1.0, 1.0);
    const Vector v3 = Vector(-1.0, 1.0,-1.0);
    const Vector v4 = Vector( 1.0, 1.0, 1.0);

    return Normalize(
                     v1 * Evaluate( pos + v1*eps ) +
                     v2 * Evaluate( pos + v2*eps ) +
                     v3 * Evaluate( pos + v3*eps ) +
                     v4 * Evaluate( pos + v4*eps ) );
}


bool DistanceEstimator::Intersect(const Ray &r, float *tHit, float *rayEpsilon,
                       DifferentialGeometry *dg) const {
    // Transform _Ray_ to object space
    Ray ray;
    (*WorldToObject)(r, &ray);

    float dist, t = ray.mint;
    Point phit = ray(t);
    int step = 0;
    while ((dist = Evaluate(phit)) > params.hitEpsilon) {
        t += dist;
        if (t > ray.maxt || ++step > params.maxIters || isinf(t)) {
            return false;
        }
        phit = ray(t);
    }
    
    *tHit = t;
    *rayEpsilon = params.rayEpsilonMultiplier * params.hitEpsilon;
    
    Vector n = CalculateNormal(phit, params.normalEpsilon);
    Vector dpdu, dpdv;
    if (fabsf(n.x) > fabsf(n.y)) {
        float invLen = 1.f / sqrtf(n.x*n.x + n.z*n.z);
        dpdu = Vector(-n.z * invLen, 0.f, n.x * invLen);
    }
    else {
        float invLen = 1.f / sqrtf(n.y*n.y + n.z*n.z);
        dpdu = Vector(0.f, n.z * invLen, -n.y * invLen);
    }
    dpdv = Cross(n, dpdu);
    const Transform &o2w = *ObjectToWorld;
    *dg = DifferentialGeometry(o2w(phit), o2w(dpdu), o2w(dpdv),
                              Normal(0, 0, 0), Normal(0, 0, 0), 0, 0, this);
    return true;
}


bool DistanceEstimator::IntersectP(const Ray &r) const {
    Ray ray;
    (*WorldToObject)(r, &ray);
    
    float dist, t = ray.mint;
    Point phit = ray(t);
    int step = 0;
    while ((dist = Evaluate(phit)) > params.hitEpsilon) {
        t += dist;
        if (t > ray.maxt || ++step > params.maxIters || isinf(t)) {
            return false;
        }
        phit = ray(t);
    }
    return true;
}

