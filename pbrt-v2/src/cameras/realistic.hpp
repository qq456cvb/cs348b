//
//  realistic.hpp
//  pbrt
//
//  Created by Neil on 02/10/2016.
//
//

#ifndef realistic_hpp
#define realistic_hpp

#include <stdio.h>
#include "pbrt.h"
#include "camera.h"
#include "film.h"
#include <vector>

struct Lense {
    float lens_radius;
    float thickness;
    float refraction;
    float aperture; // diameter
};

// RealisticCamera Declarations
class RealisticCamera : public Camera {
public:
    // PerspectiveCamera Public Methods
    RealisticCamera(const AnimatedTransform &cam2world, float shutteropen,
                    float shutterclose, float filmdistance, float fstop,
                    string specfile, float filmdiag, Film *film);
    float GenerateRay(const CameraSample &sample, Ray *) const;
private:
    // PerspectiveCamera Private Data
    Vector dxCamera, dyCamera;
    std::vector<Lense> lenses;
    float film_distance;
    float aperture_diameter;
    float film_diag;
    
    void RasterToCamera(const Point &pras, Point *pfilm) const;
};


RealisticCamera *CreateRealisticCamera(const ParamSet params,
                                       const AnimatedTransform &cam2world, Film *film);

#endif /* realistic_hpp */
