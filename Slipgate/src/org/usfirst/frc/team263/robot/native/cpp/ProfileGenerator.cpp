#include <math.h>
#include <malloc.h>
#include "ProfileGenerator.h"

#define MIN(a,b)  (((a)<(b)) ? (a):(b))
#define MAX(a,b)  (((a)>(b)) ? (a):(b))

/**
	Generate profiles for motion profiling using a double linear two filter method.
	Outline of method can be found at https://www.chiefdelphi.com/forums/showpost.php?p=1204107&postcount=18

	@author Dan Waxman
	@version 0.1 01/29/2018
*/


/**
	Constructs a new profile generator.

	@param itp Time per iteration.
	@param t1 Time for first filter.
	@param t2 Time for second filter.
	@param vprog Programmed maxable speed.
	@param dist Distance to travel.
*/
ProfileGenerator::ProfileGenerator(double itp, double t1, double t2, double vprog, double dist)
{
	fl1 = ceil(t1 / itp);
	fl2 = ceil(t2 / itp);

	t4 = dist / vprog;
	n = ceil(t4 / itp);

	f2 = (double*)malloc(fl2 * sizeof(double));
	for (int i = 0; i < fl2; i++) {
		f2[i] = 0;
	}
	timestep = 0;
	f1s = 0;
	f2s = 0;

	v = vprog;
}

/**
	Frees memory from pointers.
*/
ProfileGenerator::~ProfileGenerator()
{
	free(f2);
}

/**
	Generates a full s-curve profile.

	@return A double pointer containing velocity commands, with the first entry being the length of the pointer.
*/
double* ProfileGenerator::getProfile()
{
	double* prof = (double*)malloc(2 * n * sizeof(double) + 1);
	
	prof[0] = 2 * n + 1;

	for (int i = 1; i < 2 * n + 1; i++) {
		prof[i] = getNextCommand();
	}

	return prof;
}

/**
	Returns the target velocity for the next timestep of the motion profiler. Also advances timestep.

	@return Target velocity of profile for next timestep.
*/
double ProfileGenerator::getNextCommand()
{
	timestep++;
	if (timestep >= 2 * n) return 0;

	// Fill the first filter with an input of 1 if the step is less than or equal to n.
	if (f1i < fl1 && timestep <= n) {
		f1s = MIN(f1s + 1, fl1);
	}

	// Reset the filter index to 0 if we're at the nth timestep.
	// This prepares the array for an input of 0.
	if (timestep == n) f1i = 0;

	// Fill the first filter with an input of 0 if the step is greater than n.
	if (f1i < fl1 && timestep > n) {
		f1s = MAX(f1s - 1, 0);
		f1i++;
	}

	// Record the weighted value of filter 1 to add in the end.
	double f1st = f1s / fl1;

	// Right shift filter 2 while calculating the new sum.
	double input = ((double)f1s) / ((double)fl1);
	f2s = input;
	for (int i = 0; i < fl2 - 1; i++) {
		double temp = f2[i];
		f2s += temp;
		f2[i] = input;
		input = f2[i + 1];
		f2[i + 1] = temp;
	}

	// Return the velocity times the output ratio (based on filters).
	return v * (f1s + f2s + f1st) / (fl1 + fl2 + 1);
}
