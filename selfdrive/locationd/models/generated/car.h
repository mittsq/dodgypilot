#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2571179186929300155);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_890104746450149781);
void car_H_mod_fun(double *state, double *out_6961251219028687228);
void car_f_fun(double *state, double dt, double *out_9011000529198898256);
void car_F_fun(double *state, double dt, double *out_359320948140920634);
void car_h_25(double *state, double *unused, double *out_3028591978800468047);
void car_H_25(double *state, double *unused, double *out_6270028425398156720);
void car_h_24(double *state, double *unused, double *out_6118171530882088318);
void car_H_24(double *state, double *unused, double *out_8447242849005306693);
void car_h_30(double *state, double *unused, double *out_7536879444155034384);
void car_H_30(double *state, double *unused, double *out_8788361383905405347);
void car_h_26(double *state, double *unused, double *out_4247295893986636371);
void car_H_26(double *state, double *unused, double *out_2528525106524100496);
void car_h_27(double *state, double *unused, double *out_8030265246461313692);
void car_H_27(double *state, double *unused, double *out_432431216529876389);
void car_h_29(double *state, double *unused, double *out_8305459308745819581);
void car_H_29(double *state, double *unused, double *out_2252563439584940706);
void car_h_28(double *state, double *unused, double *out_1631223729047464546);
void car_H_28(double *state, double *unused, double *out_4216193711150266957);
void car_h_31(double *state, double *unused, double *out_6921491413645343577);
void car_H_31(double *state, double *unused, double *out_1902317004290749020);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}