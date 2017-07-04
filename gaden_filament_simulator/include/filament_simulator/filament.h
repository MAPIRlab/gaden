#ifndef FILAMENT_H
#define FILAMENT_H


class CFilament
{
public:
    CFilament();
    CFilament(double x, double y, double z, double sigma_filament);
    ~CFilament();
    void activate_filament(double x, double y, double z, double birth);
    void deactivate_filament();

    //Parameters of the filament
    //--------------------------        
    double pose_x;          // Center of the filament (m)
    double pose_y;          // Center of the filament (m)
    double pose_z;          // Center of the filament (m)
    double sigma;           // [cm] The sigma of a 3D gaussian (controlls the shape of the filament)
    bool valid;             // Is filament valid?
    double birth_time;      // Time at which the filament is released (set as active)
};
#endif
