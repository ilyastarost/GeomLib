#pragma once
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
	};
}