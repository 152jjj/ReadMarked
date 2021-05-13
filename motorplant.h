#ifndef MOTOR_PLANT_H
#define MOTOR_PLANT_H
struct InductionMachineSimulated {
	double x[13]; ////////////////////////////////
	double rpm;
	double rpm_cmd;
	double rpm_deriv_cmd;
	double Tload;
	double Tem;

	double Lsigma;
	double rs;
	double rreq;
	double Lmu;
	double Lmu_inv;
	double alpha;

	double Js;
	double npp;
	double mu_m;
	double Ts;

	double iqs;
	double ids;

	double ual;
	double ube;
};
extern struct InductionMachineSimulated IM;
void IM_init();
int machine_simulation();
void rK555_Lin(double t, double *x, double hs);
void rK5_dynamics(double t, double *x, double *fx);
void measurement();
#endif
