
#include "freertos/portmacro.h"

typedef struct {
    double azimuth;
    double elevation;
    double distance;
} AdmPolar_t;

typedef struct {
    double x;
    double y;
    double z;
} AdmCartesian_t;

void azim(float azimuth);
void elev(float elevation);
void dist(float distance);
void aed(float azimuth, float elevation, float distance);

void x(float x_position);
void y(float y_position);
void z(float z_position);
void xy(float x_position, float y_position);
void xyz(float x_position, float y_position, float z_position);

void dmax(float dmax);
float get_dmax(void);
BaseType_t get_coords_from_parameters(AdmPolar_t *polar, AdmCartesian_t *cart);