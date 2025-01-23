#include "source/Point.h"
#include <iostream>

int main()
{
	geomlib::Point2D<double> q(1, 2), q2(2, 5);
	geomlib::Vector2D<double> v(3, 3);

	std::cout << q.dist_to(q2) << std::endl;

	geomlib::Point2D<double> nw = q + v;
	std::cout << nw.X() << ' ' << nw.Y() << std::endl;

	if (q == q) std::cout << "True" << std::endl;
	else std::cout << "False" << std::endl;
	if (q != q2) std::cout << "True" << std::endl;
	else std::cout << "False" << std::endl;

	q = geomlib::Point2D<double>(3, 8);
	geomlib::Point2D<float> w;
	std::cout << q.X() << ' ' << q.Y() << std::endl;

	q += v;
	std::cout << q.X() << ' ' << q.Y() << std::endl;
}
