#ifndef FILAMENT_H
#define FILAMENT_H

#include "gaden_common/Vector3.h"

class CFilament
{
public:
    CFilament();
    CFilament(float x, float y, float z, float sigma_filament);
    ~CFilament();
    void activate_filament(float x, float y, float z, float birth);
    void deactivate_filament();

    // Parameters of the filament
    //--------------------------
    gaden::Vector3 pose;
    float sigma;      // [cm] The sigma of a 3D gaussian (controlls the shape of the filament)
    bool valid;        // Is filament valid?
    float birth_time; // Time at which the filament is released (set as active)
};
#endif
