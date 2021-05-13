
#include "ACMSim.h"
#define VVVF_CONTROL 1
int main()
{
	printf("NUMBER_OF_LINES:%d\n\n", NUMBER_OF_LINES );

	//Initialization
	IM_init();

	CTRL_init();

	FILE *fw;
	fw = fopen("algorithm.dat", "w");

	clock_t  begin, end;
	begin = clock();
	int _; // _ for the outer iteration
	int dfe = 0; // dfe for down frequency execution
	for (_ = 0; _ < NUMBER_OF_LINES; ++_) {

		/* Command and Load Torque */
		IM.rpm_cmd = 50; // rpm
		IM.Tload = 10; // Nm

		/* Simulated IM */
		if (machine_simulation()) {
			printf("Break the loop.\n");
			break;
		}

		if (++dfe == DOWN_FREQ_EXE) {
			dfe = 0;

// 			/* Time */
			CTRL.timebase += TS;

			measurement();

			write_data_to_file(fw);

#if VVVF_CONTROL == true
#define VF_RATIO 18 //18.0 // 8 ~ 18 shows saturated phenomenon
			double freq = 2; // 0.15 ~ 0.5 ~ 2 （0.1时电压李萨茹就变成一个圆了）
			double volt = VF_RATIO * freq;
			CTRL.ual = volt * cos(2 * M_PI * freq * CTRL.timebase);
			CTRL.ube = volt * sin(2 * M_PI * freq * CTRL.timebase);
#else
			//	control();
#endif
		}

		inverter_model();
	}
	end = clock(); printf("The simulation in C costs %g sec.\n", (double)(end - begin) / CLOCKS_PER_SEC);
	fclose(fw);

	/* Fade out */
	system("python ./ACMPlot.py");
	// getch();

	return 0;
}

bool isNumber(double x) {
	// This looks like it should always be true,
	// but it's false if x is a NaN (1.#QNAN0).
	return (x == x);
	// see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}
/* Utility */
void write_data_to_file(FILE *fw) {
	static int j = 0, jj = 0; // j,jj for down sampling

	// if(CTRL.timebase>20)
	{
		if (++j == 10)
		{
			j = 0;
			fprintf(fw, "%g,%g,%g,%g,%g\n",
			        IM.x[0], IM.x[1], IM.x[2], IM.x[3], IM.x[4] / IM.npp / 2 / M_PI * 60 // from elec.rad/s to rpm
			       );
		}
	}
}
void inverter_model() {
	IM.ual = CTRL.ual;
	IM.ube = CTRL.ube;
}

