#ifndef ALT_FILTER_H
#define ALT_FILTER_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include "Fusion.h"
#include "defs.h"

typedef struct altimetry_filter_t {
    void* Kf; // (KalmanFilter*)
    bool is_init;
} altimetry_filter_t;


EXTERNC void altimetry_filter_init(altimetry_filter_t* filter, float dt, float vertical_acceleration_stdev, float altimetry_stdev);
EXTERNC void altimetry_filter_update(altimetry_filter_t* filter, FusionAhrs* ahrs);
EXTERNC void altimetry_filter_correct(altimetry_filter_t* filter, float baro_altitude);

EXTERNC float altimetry_filter_get_filtered_vspeed(altimetry_filter_t* filter);
EXTERNC float altimetry_filter_get_filtered_altitude(altimetry_filter_t* filter);


#undef EXTERNC

#endif