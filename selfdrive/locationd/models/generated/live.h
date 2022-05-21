#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6048962248150004469);
void live_err_fun(double *nom_x, double *delta_x, double *out_1257180605249194379);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7197721205927284659);
void live_H_mod_fun(double *state, double *out_4220929202088409045);
void live_f_fun(double *state, double dt, double *out_3310092106447953143);
void live_F_fun(double *state, double dt, double *out_8347382080481440720);
void live_h_4(double *state, double *unused, double *out_982242475321289129);
void live_H_4(double *state, double *unused, double *out_9051687555922707218);
void live_h_9(double *state, double *unused, double *out_4530119351749384424);
void live_H_9(double *state, double *unused, double *out_8810497909293116573);
void live_h_10(double *state, double *unused, double *out_727747743377114058);
void live_H_10(double *state, double *unused, double *out_2500188153629558271);
void live_h_12(double *state, double *unused, double *out_578281573732598570);
void live_H_12(double *state, double *unused, double *out_4032231147890745423);
void live_h_31(double *state, double *unused, double *out_4597987201325971349);
void live_H_31(double *state, double *unused, double *out_1286668115565731714);
void live_h_32(double *state, double *unused, double *out_2627031169915381729);
void live_H_32(double *state, double *unused, double *out_1496990048852685879);
void live_h_13(double *state, double *unused, double *out_1464207722452825035);
void live_H_13(double *state, double *unused, double *out_2609377365523092716);
void live_h_14(double *state, double *unused, double *out_4530119351749384424);
void live_H_14(double *state, double *unused, double *out_8810497909293116573);
void live_h_33(double *state, double *unused, double *out_5073345170562244081);
void live_H_33(double *state, double *unused, double *out_1863888889073125890);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}