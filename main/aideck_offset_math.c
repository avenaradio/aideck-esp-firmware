/* Drone x = adm y
   Drone y = adm -x
ADM:
    --------y---------
    |       ^        |  
    |       |        |
  (-x)------o------->x  
    |       |        |  
    |       |        |  
    ------(-y)-------- 

Drone:
(x, y) --------- (x, -y)  
   |                |  
   |       x        |  
   |                |  
   |                |  
(-x, y) ---------(-x, -y) 

Drone coords
|
offset
normalize
convert to polar if needed
|
merge with new ADM-OSC
|
convert polar to xyz
get absolute values
offset
|
Drone coords
*/

#include <math.h>

#include "aideck_global_parameters.h"
#include "esp_log.h"
#include "freertos/projdefs.h"
#define TAG "OFFSET_MATH"

// Coords of drone when at head position in m
#define X_OFFSET 0.4295f
#define Y_OFFSET 0.1324f
#define Z_OFFSET 1.0f
// ADM-OSC orientation, x=1 is equivalent to X_WIDTH in m
#define X_WIDTH 1.1f
#define Y_WIDTH 0.8f
#define Z_WIDTH Z_OFFSET

#define ADM_DEG_TO_RAD (M_PI / 180.0)
#define ADM_RAD_TO_DEG (180.0 / M_PI)

/*
 * ADM-OSC polar coordinates:
 *   azimuth   : degrees, 0° straight ahead, positive to the left
 *   elevation : degrees, +90° upwards
 *   distance  : normalized radius, 0.0 to 1.0
 */
typedef struct {
    double azimuth;
    double elevation;
    double distance;
} AdmPolar_t;

/*
 * ADM-OSC Cartesian coordinates:
 *   x = right
 *   y = forward
 *   z = up
 * Each coordinate is normally in the range [-1.0, 1.0].
 */
typedef struct {
    double x;
    double y;
    double z;
} AdmCartesian_t;

AdmPolar_t adm_polar = {0};
AdmCartesian_t adm_cartesian = {0};

// Function prototypes
BaseType_t adm_polar_to_cartesian(AdmPolar_t *polar, AdmCartesian_t *cart);
BaseType_t adm_cartesian_to_polar(AdmCartesian_t *cart, AdmPolar_t *polar);
BaseType_t get_coords_from_parameters(AdmPolar_t *polar, AdmCartesian_t *cart);
BaseType_t set_goto_from_cartesian(AdmCartesian_t *cart);

// -------------------------------- ADM-OSC processing ----------------------------------------- //
void azim(float azimuth) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_polar.azimuth = azimuth;
    adm_polar_to_cartesian(&adm_polar, &adm_cartesian);
    set_goto_from_cartesian(&adm_cartesian);
}

void elev(float elevation) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_polar.elevation = elevation;
    adm_polar_to_cartesian(&adm_polar, &adm_cartesian);
    set_goto_from_cartesian(&adm_cartesian);
}

void dist(float distance) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_polar.distance = distance;
    adm_polar_to_cartesian(&adm_polar, &adm_cartesian);
    set_goto_from_cartesian(&adm_cartesian);
}

void aed(float azimuth, float elevation, float distance) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_polar.azimuth = azimuth;
    adm_polar.elevation = elevation;
    adm_polar.distance = distance;
    adm_polar_to_cartesian(&adm_polar, &adm_cartesian);
    set_goto_from_cartesian(&adm_cartesian);
}

void x(float x_position) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_cartesian.x = x_position;
    set_goto_from_cartesian(&adm_cartesian);
}

void y(float y_position) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_cartesian.y = y_position;
    set_goto_from_cartesian(&adm_cartesian);
}

void z(float z_position) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_cartesian.z = z_position;
    set_goto_from_cartesian(&adm_cartesian);
}

void xy(float x_position, float y_position) {
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    adm_cartesian.x = x_position;
    adm_cartesian.y = y_position;
    set_goto_from_cartesian(&adm_cartesian);
}

void xyz(float x_position, float y_position, float z_position) {
    ESP_LOGI(TAG, "Starting xyz() with x=%f y=%f z=%f", x_position, y_position, z_position);
    get_coords_from_parameters(&adm_polar, &adm_cartesian);
    ESP_LOGI(TAG, "Loaded from parameters x=%f y=%f z=%f", adm_cartesian.x, adm_cartesian.y, adm_cartesian.z);
    ESP_LOGI(TAG, "Loaded from parameters a=%f e=%f d=%f", adm_polar.azimuth, adm_polar.elevation, adm_polar.distance);
    adm_cartesian.x = x_position;
    adm_cartesian.y = y_position;
    adm_cartesian.z = z_position;
    set_goto_from_cartesian(&adm_cartesian);
}

//--------------------------------------- HELPER FUNCTIONS --------------------------------------------//

/*
 * Clamp a value to the given range.
 */
static double clamp_double(double value, double min, double max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/*
 * Cartesian -> ADM-OSC polar
 * https://immersive-audio-live.github.io/ADM-OSC/#conversions
 */
BaseType_t adm_cartesian_to_polar(AdmCartesian_t *cart, AdmPolar_t *polar){
    if (polar == NULL || cart == NULL) {return pdFALSE;}
    const double x = cart->x;
    const double y = cart->y;
    const double z = cart->z;

    const double distance = sqrt(x * x + y * y + z * z);

    if (distance <= 0.0) {
        polar->azimuth = 0.0;
        polar->elevation = 0.0;
        polar->distance = 0.0;
        return pdTRUE;
    }

    /*
     * The clamp protects against small floating-point errors that could
     * otherwise make asin() receive a value just outside [-1, 1].
     */
    const double elevation_argument = clamp_double(z / distance, -1.0, 1.0);
    polar->elevation = asin(elevation_argument) * ADM_RAD_TO_DEG;
    polar->azimuth = atan2(x, y) * ADM_RAD_TO_DEG;
    polar->distance = distance;
    return pdTRUE;
}


/*
 * ADM-OSC polar -> Cartesian
 * https://immersive-audio-live.github.io/ADM-OSC/#conversions
 */
BaseType_t adm_polar_to_cartesian(AdmPolar_t *polar, AdmCartesian_t *cart){
    if (polar == NULL || cart == NULL) {return pdFALSE;}
    const double azimuth = polar->azimuth;
    const double elevation = polar->elevation;
    const double distance = polar->distance;

    const double azimuth_rad = azimuth * ADM_DEG_TO_RAD;
    const double elevation_rad = elevation * ADM_DEG_TO_RAD;
    const double co_latitude = (M_PI / 2.0) - elevation_rad;

    cart->x = -distance * sin(co_latitude) * sin(azimuth_rad);
    cart->y =  distance * sin(co_latitude) * cos(azimuth_rad);
    cart->z =  distance * cos(co_latitude);
    return pdTRUE;
}


/*
 * Reads drone coords from parameters and calculates adm coords in cartesian and polar
 */
BaseType_t get_coords_from_parameters(AdmPolar_t *polar, AdmCartesian_t *cart){
    if (polar == NULL || cart == NULL) {return pdFALSE;}
    Parameters_t parameters = {0};
    if (parameters_get(&parameters) != pdPASS) {
            return pdFALSE;
    }
    // Offset
    float x = parameters.x - X_OFFSET;
    float y = parameters.y - Y_OFFSET;
    float z = parameters.z - Z_OFFSET;
    // Normalize
    cart->x = x / X_WIDTH;
    cart->y = y / Y_WIDTH;
    cart->z = z / Z_WIDTH;

    // Also get the polar coords
    if (adm_cartesian_to_polar(cart, polar) == pdTRUE){
        return pdTRUE;
    } else {
        return  pdFALSE;
    }
}

/*
 * Takes adm cartesian coords and sends drone goto fixed position
 */
BaseType_t set_goto_from_cartesian(AdmCartesian_t *cart){
    //ESP_LOGI(TAG, "set_goto_from_cartesian() started with: x=%f y=%f z=%f", cart->x, cart->y, cart->z);
    if (cart == NULL) {return pdFALSE;}
    GoToFixPosition_t new_position = {0};
    // Denormalize
    float x = cart->x * X_WIDTH;
    float y = cart->y * Y_WIDTH;
    float z = cart->z * Z_WIDTH;
    //ESP_LOGI(TAG, "set_goto_from_cartesian() denormalized: x=%f y=%f z=%f", x, y, z);
    //Turn xy +90°(left) & offset
    new_position.x = x + X_OFFSET;
    new_position.y = y + Y_OFFSET;
    new_position.z = z + Z_OFFSET;
    //ESP_LOGI(TAG, "set_goto_from_cartesian() minus offset: x=%f y=%f z=%f", new_position.x, new_position.y, new_position.z);
    BaseType_t result = goto_fix_position_set(&new_position);
    return result;
}