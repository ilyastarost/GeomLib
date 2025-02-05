#pragma once
#include <cmath>

namespace geomlib
{
	class Epsilon {
	private:
		static double eps;
		static double epsPow2;
	public:
		static void SetEpsilon(const double& val)
		{
			eps = val;
			epsPow2 = eps * eps;
		}
		static double Eps()
		{
			return eps;
		}
		static double EpsPow2()
		{
			return epsPow2;
		}
		static bool IsZero(double num)
		{
			return abs(num) <= eps;
		}
	};

	double Epsilon::eps = 1e-6;
	double Epsilon::epsPow2 = 1e-12;
}