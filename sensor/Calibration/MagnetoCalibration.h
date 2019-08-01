#ifndef _MAGNETO_CALIBRATION_H_
#define _MAGNETO_CALIBRATION_H_


typedef enum _calibrate_return_ {
    calibrate_return_ok,
    calibrate_return_error,
    calibrate_return_cancelled
}calibrate_return;


int ellipsoid_fit_least_squares(const float x[], const float y[], const float z[],
    int size, int max_iterations, float delta, float *offset_x, float *offset_y, float *offset_z,
    float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z);

calibrate_return check_calibration_result(float offset_x, float offset_y, float offset_z,
    float sphere_radius,
    float diag_x, float diag_y, float diag_z,
    float offdiag_x, float offdiag_y, float offdiag_z);


#endif

