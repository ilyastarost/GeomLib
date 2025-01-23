#include "source/Point.h"
#include <iostream>

int main()
{
	double r[3] = { 1, 2, 3 };
	geomlib::Coordinates<double> q(r);
	std::cout << q.X() << ' ' << q.Y() << ' ' << q.Z() << std::endl;
}
