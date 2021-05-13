#ifndef ACMSIM_H
#define ACMSIM_H
#include "stdio.h"
#include "time.h"
#include "motorplant.h"
#include "observer.h"
#include "controller.h"
#include "math.h"
#include <stdbool.h> // bool for _Bool and true for 1
#define NUMBER_OF_LINES (200000)
#define IM_TS 1.25e-4
#define DOWN_FREQ_EXE 2
#define TS (IM_TS*DOWN_FREQ_EXE) //2.5e-4 


bool isNumber(double x);
void write_data_to_file(FILE *fw);
void inverter_model();
#endif
