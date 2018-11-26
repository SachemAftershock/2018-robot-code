#include <jni.h>
#include "org_usfirst_frc_team263_robot_ProfileGeneratorJNI.h"
#include <stdlib.h>
#include <math.h>
#include <malloc.h>

#define MIN(a,b)  (((a)<(b)) ? (a):(b))
#define MAX(a,b)  (((a)>(b)) ? (a):(b))

JNIEXPORT jdoubleArray JNICALL Java_org_usfirst_frc_team263_robot_ProfileGeneratorJNI_createNewProfile
(JNIEnv *env, jclass jcl, jdouble itp, jdouble t1, jdouble t2, jdouble vprog, jdouble dist)
{
	int fl1 = ceil(t1 / itp);
	int fl2 = ceil(t2 / itp);

	double t4 = dist / vprog;
	double n = ceil(t4 / itp);

	double* f2 = (double*)malloc(fl2 * sizeof(double));
	for (int i = 0; i < fl2; i++) {
		f2[i] = 0;
	}
	int timestep = 0;
	double f1s = 0;
	double f2s = 0;
	int f1i = 0;
	int f2i = 0;

	double v = vprog;

	double* prof = (double*)malloc((int)(2 * n * sizeof(double) + 1));

	prof[0] = 2 * n + 1;

	for (int i = 1; i < 2 * n + 1; i++) {
		timestep++;
		if (timestep >= 2 * n) break;

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
		for (int j = 0; j < fl2 - 1; j++) {
			double temp = f2[j];
			f2s += temp;
			f2[j] = input;
			input = f2[j + 1];
			f2[j + 1] = temp;
		}

		// Return the velocity times the output ratio (based on filters).
		prof[i] = v * (f1s + f2s + f1st) / (fl1 + fl2 + 1);
	}
	// Create a new jdoublearray for the return. pg->getProfile
	// returns an array with the length as element 0.
	jdoubleArray profileJArray = env->NewDoubleArray((int)2 * n + 1);
	env->SetDoubleArrayRegion(profileJArray, 0, (int)2*n+1, prof);       

	return profileJArray;
}
