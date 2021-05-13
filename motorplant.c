
#include "ACMSim.h"
struct InductionMachineSimulated IM;
void IM_init() {
	int i;
	for (i = 0; i < 5; ++i) {
		IM.x[i] = 0.0;
	}
	IM.rpm = 0.0;

	IM.iqs = 0.0;
	IM.ids = 0.0;

	IM.Tload = 0.0;
	IM.rpm_cmd = 0.0;
	IM.rpm_deriv_cmd = 0.0;

	IM.Lmu    = 0.4482;
	IM.Lsigma = 0.0126;

	IM.rreq   = 1.69;
	IM.rs     = 3.04;

	IM.alpha  = IM.rreq / (IM.Lmu);
	IM.Lmu_inv = 1.0 / IM.Lmu;

	IM.Js = 0.0636; // Awaya92 using im.omg
	IM.npp = 2;
	IM.mu_m = IM.npp / IM.Js;

	IM.Ts  = IM_TS;

	IM.ual = 0.0;
	IM.ube = 0.0;
}
int machine_simulation() {
	rK555_Lin(CTRL.timebase, IM.x, IM.Ts);

	IM.ids = IM.x[0];
	IM.iqs = IM.x[1];
	IM.rpm = IM.x[4] * 60 / (2 * M_PI * IM.npp);

	if (isNumber(IM.rpm))
		return false;
	else
		return true;
}
void rK555_Lin(double t, double *x, double hs) {
	double k1[5], k2[5], k3[5], k4[5], xk[5];
	double fx[5];
	int i;

	rK5_dynamics(t, x, fx); // timer.t,
	for (i = 0; i < 5; ++i) {
		k1[i] = fx[i] * hs;
		xk[i] = x[i] + k1[i] * 0.5;
	}

	rK5_dynamics(t, xk, fx); // timer.t+hs/2.,
	for (i = 0; i < 5; ++i) {
		k2[i] = fx[i] * hs;
		xk[i] = x[i] + k2[i] * 0.5;
	}

	rK5_dynamics(t, xk, fx); // timer.t+hs/2.,
	for (i = 0; i < 5; ++i) {
		k3[i] = fx[i] * hs;
		xk[i] = x[i] + k3[i];
	}

	rK5_dynamics(t, xk, fx); // timer.t+hs,
	for (i = 0; i < 5; ++i) {
		k4[i] = fx[i] * hs;
		x[i] = x[i] + (k1[i] + 2 * (k2[i] + k3[i]) + k4[i]) / 6.0;
	}
}
void rK5_dynamics(double t, double *x, double *fx) {
	// electromagnetic model
	fx[2] = IM.rreq * x[0] - IM.alpha * x[2] - x[4] * x[3];
	fx[3] = IM.rreq * x[1] - IM.alpha * x[3] + x[4] * x[2];
	fx[0] = (IM.ual - IM.rs * x[0] - fx[2]) / IM.Lsigma;
	fx[1] = (IM.ube - IM.rs * x[1] - fx[3]) / IM.Lsigma;

	// mechanical model
	IM.Tem = IM.npp * (x[1] * x[2] - x[0] * x[3]);
	fx[4] = (IM.Tem - IM.Tload) * IM.mu_m;
}
void measurement() {
	US_C(0) = CTRL.ual;
	US_C(1) = CTRL.ube;
	US_P(0) = US_C(0);
	US_P(1) = US_C(1);

	IS_C(0) = IM.ids;
	IS_C(1) = IM.iqs;

	im.omg = IM.x[4];
}
