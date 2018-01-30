#pragma once

class ProfileGenerator {
public:
	ProfileGenerator(double itp, double t1, double t2, double vprog, double dist);
	~ProfileGenerator();
	double getNextCommand();
	double* getProfile();

private:
	double t4, n, f1s, f2s, v;
	int fl1, fl2, f1i, f2i, timestep;
	double* f2;
};
