/*---------------------------------------------------------------------------------------
 * Very simple implementation of the class Filament
 * A gas source releases patches/puffs of gas. Each pach or puff is composed of N filaments.
 * A filament is a 3D shape which contains Q molecules of gas.
 * The filament is affected by advection (i.e its center moves with the wind field)
 * But also for difussion (its width increases with time), and some estochastic process (random behaviour)
 * This three effects can be related to the size of the wind turbulent eddies
 * See: Filament-Based Atmospheric DispersionModel to Achieve Short Time-Scale Structure of Odor Plumes, Farrell et al, 2002
 ---------------------------------------------------------------------------------------*/


#include "filament_simulator/filament.h"

//Default Constructor
CFilament::CFilament()
{
    //Create a new filament!
    pose_x = 0.0;               //[m] Filament center pose
    pose_y = 0.0;               //[m] Filament center pose
    pose_z = 0.0;               //[m] Filament center pose
    sigma = 0.01;               //[cm] The sigma of a 3D gaussian (controlls the shape of the filament)
    birth_time = 0.0;
    valid = false;
}


//Overload Constructor
CFilament::CFilament(double x, double y, double z, double sigma_filament)
{
    //Create a new filament!
    pose_x = x;                 //[m] Filament center pose
    pose_y = y;                 //[m] Filament center pose
    pose_z = z;                 //[m] Filament center pose
    sigma = sigma_filament;     //[cm] The sigma of a 3D gaussian (controlls the shape of the filament)
    birth_time = 0.0;
    valid = false;
}

CFilament::~CFilament()
{
}

void CFilament::activate_filament(double x, double y, double z, double birth)
{
    //Active the filament at given location
    pose_x = x;
    pose_y = y;
    pose_z = z;
    birth_time = birth;
    valid = true;
}

void CFilament::deactivate_filament()
{
    //de-Active the filament
    valid = false;
}
