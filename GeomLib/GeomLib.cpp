#include "source/Segment.h"
#include "source/Line.h"
#include "source/Ray.h"
#include <iostream>

int main()
{
	geomlib::Point<double> s1(-1, -3, -1), s2(2, 5, 3);
	geomlib::Vector<double> v1(1, 2, 1), v2(-1, -4, -2);
	geomlib::Line<double> l1(s1, v1), l2(s2, v2);
	std::cout << l1.Belongs(s1 + v1 * 3) << std::endl;
	std::cout << l1.Belongs(s1 + 3 * v1) << std::endl;
	std::cout << l1.Belongs(s2) << std::endl;
	std::cout << l1.Direction().X() << std::endl;
	std::cout << l1.DistanceToLine(s2) << std::endl;
	std::cout << l1.DistanceToLinePow2(s2) << std::endl;
	std::cout << l1.DistanceToThis(s2) << std::endl;
	std::cout << l1.DistanceToThisPow2(s2) << std::endl;
	std::cout << l1.FindIntersection(l2).X() << std::endl;
	std::cout << l1.FindNearestPointToLine(s2).X() << std::endl;
	std::cout << l1.FindNearestPointToThis(s2).X() << std::endl;
	std::cout << l1.Intersects(l2) << std::endl;
	std::cout << l1.IsCollinear(l2) << std::endl;
	std::cout << l1.IsOrthogonal(l2) << std::endl;
	std::cout << l1.IsParallel(l2) << std::endl;
}
