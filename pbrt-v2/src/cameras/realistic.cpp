//
//  realistic.cpp
//  pbrt
//
//  Created by Neil on 02/10/2016.
//
//

#include "stdafx.h"
#include <fstream>
#include <sstream>
#include "geometry.h"
#include "cameras/realistic.hpp"
#include "paramset.h"
#include "sampler.h"
#include "montecarlo.h"


RealisticCamera::RealisticCamera(const AnimatedTransform &cam2world, float shutteropen,
                                 float shutterclose, float filmdistance, float fstop,
                                 string specfile, float filmdiag, Film *film)
            : Camera(cam2world, shutteropen, shutterclose, film)
{
    std::ifstream ifs(specfile);
    if (ifs.fail()) {
        Severe("lens file open error!\n");
        return;
    }
    
    for( std::string line; getline( ifs, line ); )
    {
        if (line[0] != '#') { // not comment
            std::istringstream iss(line);
            Lense lense;
            iss >> lense.lens_radius >> lense.thickness
            >> lense.refraction >> lense.aperture;

            lenses.push_back(lense);
        }
    }
    
    film_distance = filmdistance;
    aperture_diameter = fstop;
    film_diag = filmdiag;
}

void RealisticCamera::RasterToCamera(const Point &pras, Point *pfilm) const
{
    float ratio = film_diag / sqrtf(powf(film->xResolution, 2) + powf(film->yResolution, 2));
    pfilm->x = (film->xResolution / 2 - pras.x) * ratio;
    pfilm->y = (pras.y - film->yResolution / 2) * ratio;
}

float RealisticCamera::GenerateRay(const CameraSample &sample, Ray *ray) const
{
    Point Pras(sample.imageX, sample.imageY, 0);
    assert(sample.imageX >= 0 && sample.imageX <= film->xResolution+1);
    assert(sample.imageY >= 0 && sample.imageY <= film->yResolution+1);
    Point Pcamera;
    RasterToCamera(Pras, &Pcamera);
    
    float start_z = 0;
    for ( auto lense : lenses ) {
        start_z -= lense.thickness;
    }
    start_z -= film_distance;
    
    // sample from nearest intersection
    float lensU, lensV;
    ConcentricSampleDisk(sample.lensU, sample.lensV, &lensU, &lensV);
    lensU *= lenses.back().aperture / 2;
    lensV *= lenses.back().aperture / 2;
    
    float area = 3.14159 * powf(lenses.back().aperture / 2, 2);
    Ray trace_ray = Ray(Point(Pcamera.x,Pcamera.y,start_z),
                        Normalize(Vector(lensU, lensV, start_z + film_distance) - Vector(Pcamera.x, Pcamera.y, start_z)), 0.f, INFINITY);
    float cos = film_distance / (Vector(lensU, lensV, start_z + film_distance) - Vector(Pcamera.x, Pcamera.y, start_z)).Length();
    float weight = powf(cos, 4) * area / powf(film_distance, 2);
    
    float lense_z = start_z + film_distance;
    for (int i = lenses.size()-1; i >= 0; i--) {
        // (o + td - o')^2 = r * r, calculate intersection
        Lense lense = lenses[i];
        lense_z += lense.thickness;
        if (lense.lens_radius == 0) {
            float t = lense.thickness / trace_ray.d.z;
            assert(t > 0);
            Point intersect = trace_ray.o + t * trace_ray.d;
            if (fabsf(sqrtf(powf(intersect.x, 2) + powf(intersect.y, 2))) > min(lense.aperture, aperture_diameter) / 2.) {
                return 0.f;
            } else {
                continue;
            }
        }
        Point oprime = Point(0, 0, lense_z - lense.lens_radius);
        Vector ooprime = Vector(trace_ray.o - oprime);
        
        // solve quadratic equation
        float a = trace_ray.d.LengthSquared();
        float b = 2 * Dot(ooprime, trace_ray.d);
        float c = ooprime.LengthSquared() - powf(lense.lens_radius, 2);
        
        float delta = powf(b, 2) - 4 * a * c;
        if (delta < 0) { // no intersection
            return 0.f;
        }
        float t;
        float t1 = (- b + sqrtf(delta)) / (2 * a);
        float t2 = (- b - sqrtf(delta)) / (2 * a);
        if (t2 < 0) {
            t = t1;
        } else {
            t = t2;
        }
        assert(t > 0);
        
        Point intersect = trace_ray.o + t * trace_ray.d;
        if (fabsf(sqrtf(powf(intersect.x, 2) + powf(intersect.y, 2))) > lense.aperture / 2.) {
            return 0.f;
        }
        Vector normal = Normalize(intersect - oprime) * (lense.lens_radius > 0 ? -1 : 1);
        assert(normal.z < 0);
        Vector x = Normalize(Vector(0, -normal.z, normal.y));
        Vector y = Cross(x, normal);
        assert(fabsf(y.Length() - 1) < 0.01);
        
        // new basis
        Matrix4x4 basis(x.x, y.x, normal.x, 0,
                        x.y, y.y, normal.y, 0,
                        x.z, y.z, normal.z, 0,
                        0, 0, 0, 1);
        Transform transform_ntoc(basis);
        Transform transform_cton = Transform(transform_ntoc.GetInverseMatrix());
        
        Vector transform_d = transform_cton(trace_ray.d);
        Vector refract_d;
        assert(transform_d.z < 0);
        float snell = (i == 0 ? 1. : (lenses[i-1].refraction == 0 ? 1. : lenses[i-1].refraction)) / lense.refraction;
        if (transform_d.y != 0) {
            float xyratio = transform_d.x / transform_d.y;
            float sinthetay = fabsf(transform_d.y / sqrtf(powf(transform_d.y, 2) + powf(transform_d.z, 2)) / snell);
            if (sinthetay > 1) { // all reflect
                return 0.f;
            }
            // assume y = 1 after refraction
            float y_refract = transform_d.y > 0 ? 1. : -1.;
            float z_refract = -sqrtf(powf(1. / sinthetay, 2) - 1);
            refract_d = Vector(y_refract * xyratio, y_refract, z_refract);
            refract_d = Normalize(transform_ntoc(refract_d));
           
        } else if (transform_d.x != 0) {
            float yxratio = transform_d.y / transform_d.x;
            float sinthetax = fabsf(transform_d.x / sqrtf(powf(transform_d.x, 2) + powf(transform_d.z, 2)) / snell);
            if (sinthetax > 1) { // all reflect
                return 0.f;
            }
            // assume x = 1 after refraction
            float x_refract = transform_d.x > 0 ? 1. : -1.;
            float z_refract = -sqrtf(powf(1. / sinthetax, 2) - 1);
            refract_d = Vector(x_refract, x_refract * yxratio, z_refract);
            refract_d = Normalize(transform_ntoc(refract_d));
        } else {
            refract_d = trace_ray.d;
        }
        assert(refract_d.z > 0);
        
        trace_ray = Ray(intersect, refract_d, 0.f, INFINITY);
    }
    
    assert(fabsf(trace_ray.d.Length() - 1) < 0.01);
    *ray = Ray(trace_ray.o, trace_ray.d, 0.f, INFINITY);
    ray->time = sample.time;
    CameraToWorld(*ray, ray);
    ray->d = Normalize(ray->d);
    return weight;
}

RealisticCamera *CreateRealisticCamera(const ParamSet params,
                                       const AnimatedTransform &cam2world, Film *film) {
    // Extract common camera parameters from \use{ParamSet}
    float shutteropen = params.FindOneFloat("shutteropen", -1);
    float shutterclose = params.FindOneFloat("shutterclose", -1);
    
    // Realistic camera-specific parameters
    string specfile = params.FindOneString("specfile", "");
    float filmdistance = params.FindOneFloat("filmdistance", 70.0); // about 70 mm default to film
    float fstop = params.FindOneFloat("aperture_diameter", 1.0);
    float filmdiag = params.FindOneFloat("filmdiag", 35.0);
    assert(shutteropen != -1 && shutterclose != -1 && filmdistance!= -1);
    if (specfile == "") {
        Severe( "No lens spec file supplied!\n" );
    }
    return new RealisticCamera(cam2world, shutteropen, shutterclose, filmdistance, fstop,
                               specfile, filmdiag, film);
}
