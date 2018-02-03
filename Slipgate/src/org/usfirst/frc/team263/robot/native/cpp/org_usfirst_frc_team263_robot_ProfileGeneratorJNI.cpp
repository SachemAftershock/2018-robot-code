#include <jni.h>
#include "org_usfirst_frc_team263_robot_ProfileGeneratorJNI.h"
#include "ProfileGenerator.h"

JNIEXPORT jdoubleArray JNICALL Java_org_usfirst_frc_team263_robot_ProfileGeneratorJNI_createNewProfile
(JNIEnv *env, jclass jcl, jdouble itp, jdouble t1, jdouble t2, jdouble vprog, jdouble dist)
{
	// Create new profile generator object and get the profile.
	// jdouble is just a typedef of double so passing in params directly should be okay.
	ProfileGenerator *pg = new ProfileGenerator(itp, t1, t2, vprog, dist);
	double* profile = pg->getProfile();

	// Create a new jdoublearray for the return. pg->getProfile
	// returns an array with the length as element 0.
	jdoubleArray profileJArray = env->NewDoubleArray(profile[0]);
	env->SetDoubleArrayRegion(profileJArray, 0, profile[0], profile);

	delete pg;
	free(profile);

	return profileJArray;
}
