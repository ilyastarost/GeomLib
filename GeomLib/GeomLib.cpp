#include "source/Coordinates.h"
#include "source/Generic.h"
#include "source/Epsilon.h"
#include "source/Vector.h"
#include "source/Point.h"
#include <iostream>

int main()
{
	double r[3] = { 1, 2, 3 };
	geomlib::Coordinates<double> q(r);
	std::cout << q.X() << ' ' << q.Y() << ' ' << q.Z() << std::endl;
}
