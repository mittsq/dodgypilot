#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2571179186929300155) {
   out_2571179186929300155[0] = delta_x[0] + nom_x[0];
   out_2571179186929300155[1] = delta_x[1] + nom_x[1];
   out_2571179186929300155[2] = delta_x[2] + nom_x[2];
   out_2571179186929300155[3] = delta_x[3] + nom_x[3];
   out_2571179186929300155[4] = delta_x[4] + nom_x[4];
   out_2571179186929300155[5] = delta_x[5] + nom_x[5];
   out_2571179186929300155[6] = delta_x[6] + nom_x[6];
   out_2571179186929300155[7] = delta_x[7] + nom_x[7];
   out_2571179186929300155[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_890104746450149781) {
   out_890104746450149781[0] = -nom_x[0] + true_x[0];
   out_890104746450149781[1] = -nom_x[1] + true_x[1];
   out_890104746450149781[2] = -nom_x[2] + true_x[2];
   out_890104746450149781[3] = -nom_x[3] + true_x[3];
   out_890104746450149781[4] = -nom_x[4] + true_x[4];
   out_890104746450149781[5] = -nom_x[5] + true_x[5];
   out_890104746450149781[6] = -nom_x[6] + true_x[6];
   out_890104746450149781[7] = -nom_x[7] + true_x[7];
   out_890104746450149781[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6961251219028687228) {
   out_6961251219028687228[0] = 1.0;
   out_6961251219028687228[1] = 0;
   out_6961251219028687228[2] = 0;
   out_6961251219028687228[3] = 0;
   out_6961251219028687228[4] = 0;
   out_6961251219028687228[5] = 0;
   out_6961251219028687228[6] = 0;
   out_6961251219028687228[7] = 0;
   out_6961251219028687228[8] = 0;
   out_6961251219028687228[9] = 0;
   out_6961251219028687228[10] = 1.0;
   out_6961251219028687228[11] = 0;
   out_6961251219028687228[12] = 0;
   out_6961251219028687228[13] = 0;
   out_6961251219028687228[14] = 0;
   out_6961251219028687228[15] = 0;
   out_6961251219028687228[16] = 0;
   out_6961251219028687228[17] = 0;
   out_6961251219028687228[18] = 0;
   out_6961251219028687228[19] = 0;
   out_6961251219028687228[20] = 1.0;
   out_6961251219028687228[21] = 0;
   out_6961251219028687228[22] = 0;
   out_6961251219028687228[23] = 0;
   out_6961251219028687228[24] = 0;
   out_6961251219028687228[25] = 0;
   out_6961251219028687228[26] = 0;
   out_6961251219028687228[27] = 0;
   out_6961251219028687228[28] = 0;
   out_6961251219028687228[29] = 0;
   out_6961251219028687228[30] = 1.0;
   out_6961251219028687228[31] = 0;
   out_6961251219028687228[32] = 0;
   out_6961251219028687228[33] = 0;
   out_6961251219028687228[34] = 0;
   out_6961251219028687228[35] = 0;
   out_6961251219028687228[36] = 0;
   out_6961251219028687228[37] = 0;
   out_6961251219028687228[38] = 0;
   out_6961251219028687228[39] = 0;
   out_6961251219028687228[40] = 1.0;
   out_6961251219028687228[41] = 0;
   out_6961251219028687228[42] = 0;
   out_6961251219028687228[43] = 0;
   out_6961251219028687228[44] = 0;
   out_6961251219028687228[45] = 0;
   out_6961251219028687228[46] = 0;
   out_6961251219028687228[47] = 0;
   out_6961251219028687228[48] = 0;
   out_6961251219028687228[49] = 0;
   out_6961251219028687228[50] = 1.0;
   out_6961251219028687228[51] = 0;
   out_6961251219028687228[52] = 0;
   out_6961251219028687228[53] = 0;
   out_6961251219028687228[54] = 0;
   out_6961251219028687228[55] = 0;
   out_6961251219028687228[56] = 0;
   out_6961251219028687228[57] = 0;
   out_6961251219028687228[58] = 0;
   out_6961251219028687228[59] = 0;
   out_6961251219028687228[60] = 1.0;
   out_6961251219028687228[61] = 0;
   out_6961251219028687228[62] = 0;
   out_6961251219028687228[63] = 0;
   out_6961251219028687228[64] = 0;
   out_6961251219028687228[65] = 0;
   out_6961251219028687228[66] = 0;
   out_6961251219028687228[67] = 0;
   out_6961251219028687228[68] = 0;
   out_6961251219028687228[69] = 0;
   out_6961251219028687228[70] = 1.0;
   out_6961251219028687228[71] = 0;
   out_6961251219028687228[72] = 0;
   out_6961251219028687228[73] = 0;
   out_6961251219028687228[74] = 0;
   out_6961251219028687228[75] = 0;
   out_6961251219028687228[76] = 0;
   out_6961251219028687228[77] = 0;
   out_6961251219028687228[78] = 0;
   out_6961251219028687228[79] = 0;
   out_6961251219028687228[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_9011000529198898256) {
   out_9011000529198898256[0] = state[0];
   out_9011000529198898256[1] = state[1];
   out_9011000529198898256[2] = state[2];
   out_9011000529198898256[3] = state[3];
   out_9011000529198898256[4] = state[4];
   out_9011000529198898256[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_9011000529198898256[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_9011000529198898256[7] = state[7];
   out_9011000529198898256[8] = state[8];
}
void F_fun(double *state, double dt, double *out_359320948140920634) {
   out_359320948140920634[0] = 1;
   out_359320948140920634[1] = 0;
   out_359320948140920634[2] = 0;
   out_359320948140920634[3] = 0;
   out_359320948140920634[4] = 0;
   out_359320948140920634[5] = 0;
   out_359320948140920634[6] = 0;
   out_359320948140920634[7] = 0;
   out_359320948140920634[8] = 0;
   out_359320948140920634[9] = 0;
   out_359320948140920634[10] = 1;
   out_359320948140920634[11] = 0;
   out_359320948140920634[12] = 0;
   out_359320948140920634[13] = 0;
   out_359320948140920634[14] = 0;
   out_359320948140920634[15] = 0;
   out_359320948140920634[16] = 0;
   out_359320948140920634[17] = 0;
   out_359320948140920634[18] = 0;
   out_359320948140920634[19] = 0;
   out_359320948140920634[20] = 1;
   out_359320948140920634[21] = 0;
   out_359320948140920634[22] = 0;
   out_359320948140920634[23] = 0;
   out_359320948140920634[24] = 0;
   out_359320948140920634[25] = 0;
   out_359320948140920634[26] = 0;
   out_359320948140920634[27] = 0;
   out_359320948140920634[28] = 0;
   out_359320948140920634[29] = 0;
   out_359320948140920634[30] = 1;
   out_359320948140920634[31] = 0;
   out_359320948140920634[32] = 0;
   out_359320948140920634[33] = 0;
   out_359320948140920634[34] = 0;
   out_359320948140920634[35] = 0;
   out_359320948140920634[36] = 0;
   out_359320948140920634[37] = 0;
   out_359320948140920634[38] = 0;
   out_359320948140920634[39] = 0;
   out_359320948140920634[40] = 1;
   out_359320948140920634[41] = 0;
   out_359320948140920634[42] = 0;
   out_359320948140920634[43] = 0;
   out_359320948140920634[44] = 0;
   out_359320948140920634[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_359320948140920634[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_359320948140920634[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_359320948140920634[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_359320948140920634[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_359320948140920634[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_359320948140920634[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_359320948140920634[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_359320948140920634[53] = -9.8000000000000007*dt;
   out_359320948140920634[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_359320948140920634[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_359320948140920634[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_359320948140920634[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_359320948140920634[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_359320948140920634[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_359320948140920634[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_359320948140920634[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_359320948140920634[62] = 0;
   out_359320948140920634[63] = 0;
   out_359320948140920634[64] = 0;
   out_359320948140920634[65] = 0;
   out_359320948140920634[66] = 0;
   out_359320948140920634[67] = 0;
   out_359320948140920634[68] = 0;
   out_359320948140920634[69] = 0;
   out_359320948140920634[70] = 1;
   out_359320948140920634[71] = 0;
   out_359320948140920634[72] = 0;
   out_359320948140920634[73] = 0;
   out_359320948140920634[74] = 0;
   out_359320948140920634[75] = 0;
   out_359320948140920634[76] = 0;
   out_359320948140920634[77] = 0;
   out_359320948140920634[78] = 0;
   out_359320948140920634[79] = 0;
   out_359320948140920634[80] = 1;
}
void h_25(double *state, double *unused, double *out_3028591978800468047) {
   out_3028591978800468047[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6270028425398156720) {
   out_6270028425398156720[0] = 0;
   out_6270028425398156720[1] = 0;
   out_6270028425398156720[2] = 0;
   out_6270028425398156720[3] = 0;
   out_6270028425398156720[4] = 0;
   out_6270028425398156720[5] = 0;
   out_6270028425398156720[6] = 1;
   out_6270028425398156720[7] = 0;
   out_6270028425398156720[8] = 0;
}
void h_24(double *state, double *unused, double *out_6118171530882088318) {
   out_6118171530882088318[0] = state[4];
   out_6118171530882088318[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8447242849005306693) {
   out_8447242849005306693[0] = 0;
   out_8447242849005306693[1] = 0;
   out_8447242849005306693[2] = 0;
   out_8447242849005306693[3] = 0;
   out_8447242849005306693[4] = 1;
   out_8447242849005306693[5] = 0;
   out_8447242849005306693[6] = 0;
   out_8447242849005306693[7] = 0;
   out_8447242849005306693[8] = 0;
   out_8447242849005306693[9] = 0;
   out_8447242849005306693[10] = 0;
   out_8447242849005306693[11] = 0;
   out_8447242849005306693[12] = 0;
   out_8447242849005306693[13] = 0;
   out_8447242849005306693[14] = 1;
   out_8447242849005306693[15] = 0;
   out_8447242849005306693[16] = 0;
   out_8447242849005306693[17] = 0;
}
void h_30(double *state, double *unused, double *out_7536879444155034384) {
   out_7536879444155034384[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8788361383905405347) {
   out_8788361383905405347[0] = 0;
   out_8788361383905405347[1] = 0;
   out_8788361383905405347[2] = 0;
   out_8788361383905405347[3] = 0;
   out_8788361383905405347[4] = 1;
   out_8788361383905405347[5] = 0;
   out_8788361383905405347[6] = 0;
   out_8788361383905405347[7] = 0;
   out_8788361383905405347[8] = 0;
}
void h_26(double *state, double *unused, double *out_4247295893986636371) {
   out_4247295893986636371[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2528525106524100496) {
   out_2528525106524100496[0] = 0;
   out_2528525106524100496[1] = 0;
   out_2528525106524100496[2] = 0;
   out_2528525106524100496[3] = 0;
   out_2528525106524100496[4] = 0;
   out_2528525106524100496[5] = 0;
   out_2528525106524100496[6] = 0;
   out_2528525106524100496[7] = 1;
   out_2528525106524100496[8] = 0;
}
void h_27(double *state, double *unused, double *out_8030265246461313692) {
   out_8030265246461313692[0] = state[3];
}
void H_27(double *state, double *unused, double *out_432431216529876389) {
   out_432431216529876389[0] = 0;
   out_432431216529876389[1] = 0;
   out_432431216529876389[2] = 0;
   out_432431216529876389[3] = 1;
   out_432431216529876389[4] = 0;
   out_432431216529876389[5] = 0;
   out_432431216529876389[6] = 0;
   out_432431216529876389[7] = 0;
   out_432431216529876389[8] = 0;
}
void h_29(double *state, double *unused, double *out_8305459308745819581) {
   out_8305459308745819581[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2252563439584940706) {
   out_2252563439584940706[0] = 0;
   out_2252563439584940706[1] = 1;
   out_2252563439584940706[2] = 0;
   out_2252563439584940706[3] = 0;
   out_2252563439584940706[4] = 0;
   out_2252563439584940706[5] = 0;
   out_2252563439584940706[6] = 0;
   out_2252563439584940706[7] = 0;
   out_2252563439584940706[8] = 0;
}
void h_28(double *state, double *unused, double *out_1631223729047464546) {
   out_1631223729047464546[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4216193711150266957) {
   out_4216193711150266957[0] = 1;
   out_4216193711150266957[1] = 0;
   out_4216193711150266957[2] = 0;
   out_4216193711150266957[3] = 0;
   out_4216193711150266957[4] = 0;
   out_4216193711150266957[5] = 0;
   out_4216193711150266957[6] = 0;
   out_4216193711150266957[7] = 0;
   out_4216193711150266957[8] = 0;
}
void h_31(double *state, double *unused, double *out_6921491413645343577) {
   out_6921491413645343577[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1902317004290749020) {
   out_1902317004290749020[0] = 0;
   out_1902317004290749020[1] = 0;
   out_1902317004290749020[2] = 0;
   out_1902317004290749020[3] = 0;
   out_1902317004290749020[4] = 0;
   out_1902317004290749020[5] = 0;
   out_1902317004290749020[6] = 0;
   out_1902317004290749020[7] = 0;
   out_1902317004290749020[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_2571179186929300155) {
  err_fun(nom_x, delta_x, out_2571179186929300155);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_890104746450149781) {
  inv_err_fun(nom_x, true_x, out_890104746450149781);
}
void car_H_mod_fun(double *state, double *out_6961251219028687228) {
  H_mod_fun(state, out_6961251219028687228);
}
void car_f_fun(double *state, double dt, double *out_9011000529198898256) {
  f_fun(state,  dt, out_9011000529198898256);
}
void car_F_fun(double *state, double dt, double *out_359320948140920634) {
  F_fun(state,  dt, out_359320948140920634);
}
void car_h_25(double *state, double *unused, double *out_3028591978800468047) {
  h_25(state, unused, out_3028591978800468047);
}
void car_H_25(double *state, double *unused, double *out_6270028425398156720) {
  H_25(state, unused, out_6270028425398156720);
}
void car_h_24(double *state, double *unused, double *out_6118171530882088318) {
  h_24(state, unused, out_6118171530882088318);
}
void car_H_24(double *state, double *unused, double *out_8447242849005306693) {
  H_24(state, unused, out_8447242849005306693);
}
void car_h_30(double *state, double *unused, double *out_7536879444155034384) {
  h_30(state, unused, out_7536879444155034384);
}
void car_H_30(double *state, double *unused, double *out_8788361383905405347) {
  H_30(state, unused, out_8788361383905405347);
}
void car_h_26(double *state, double *unused, double *out_4247295893986636371) {
  h_26(state, unused, out_4247295893986636371);
}
void car_H_26(double *state, double *unused, double *out_2528525106524100496) {
  H_26(state, unused, out_2528525106524100496);
}
void car_h_27(double *state, double *unused, double *out_8030265246461313692) {
  h_27(state, unused, out_8030265246461313692);
}
void car_H_27(double *state, double *unused, double *out_432431216529876389) {
  H_27(state, unused, out_432431216529876389);
}
void car_h_29(double *state, double *unused, double *out_8305459308745819581) {
  h_29(state, unused, out_8305459308745819581);
}
void car_H_29(double *state, double *unused, double *out_2252563439584940706) {
  H_29(state, unused, out_2252563439584940706);
}
void car_h_28(double *state, double *unused, double *out_1631223729047464546) {
  h_28(state, unused, out_1631223729047464546);
}
void car_H_28(double *state, double *unused, double *out_4216193711150266957) {
  H_28(state, unused, out_4216193711150266957);
}
void car_h_31(double *state, double *unused, double *out_6921491413645343577) {
  h_31(state, unused, out_6921491413645343577);
}
void car_H_31(double *state, double *unused, double *out_1902317004290749020) {
  H_31(state, unused, out_1902317004290749020);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
