#include "source/Testing.h"
#include "source/Segment.h"
#include "source/Line.h"
#include "source/Ray.h"
#include <iostream>

int main()
{
	geomlib::Point<double> s1(-1, -3, -1), s2(2, 5, 3);
	geomlib::Vector<double> v1(1, 2, 1), v2(-1, -4, -2), vc(2, -1);
	geomlib::Line<double> l1(s1, v1), l2(s2, v2);

	TESTING_SECTION_OPEN;
	TEST("Points");
	SUBTEST_EQ("Equal", s1, geomlib::Point<double>(-1, -3, -1));
	SUBTEST_UNEQ("Uneqal", s1, s2);
	TEST("Vectors");
	SUBTEST_EQ("2D constructor", vc, geomlib::Vector<double>(2, -1, 0));
	SUBTEST_EQ("3D constructor", v1.Y(), 2);
	SUBTEST_EQ("Equal", s1 + 2 * v1, s2 + v2 * 1);
	SUBTEST_ASSERT("Equal func", (s1 + 2 * v1).IsEqual(s2 + v2 * 1));
	SUBTEST_ASSERT("Uneqal func", !v1.IsEqual(v2));
	TESTING_SECTION_CLOSE;

}
