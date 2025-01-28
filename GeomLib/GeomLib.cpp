#include "source/Testing.h"
#include "source/Segment.h"
#include "source/Line.h"
#include "source/Ray.h"
#include <iostream>

int main()
{
	TESTING_SECTION_OPEN;

	TEST("Multiplication");
	geomlib::Point<double> p1(1, 1, 1), p2(2, 3, -1);
	geomlib::Vector<double> v1(3, -1, 1), v2(1, 2, -1), zero(0, 0, 0);
	geomlib::Line<double> l1(p1, v1), l2(p2, v1);

	SUBTEST_EQ("Dot parallel", v1.DotProduct(2 * v1), 22);
	SUBTEST_EQ("Cross parallel", v1.CrossProduct(2 * v1), geomlib::Vector<double>(0, 0, 0));
	
	SUBTEST_EQ("Dot orthogonal", v1.DotProduct(v2), 0);
	SUBTEST_EQ("Cross orthogonal", v1.CrossProduct(v2), geomlib::Vector<double>(-1, 4, 7));

	SUBTEST_EQ("Dot zero", v1.DotProduct(zero), 0);
	SUBTEST_EQ("Cross zero", v1.CrossProduct(zero), zero);


	TEST("Belonging");
	p1 = geomlib::Point<double>(-1, 3, 2); p2 = geomlib::Point<double>(0, 2.5, 2.5);
	v1 = geomlib::Vector<double>(2, -1, 1);
	l1 = geomlib::Line<double>(p1, v1);
	geomlib::Ray<double> r1(p1, v1), r2;
	geomlib::Segment<double> s1(p1, v1), s2;

	SUBTEST_ASSERT("Start point of line", l1.Belongs(p1));
	SUBTEST_ASSERT("Start point of ray", r1.Belongs(p1));
	SUBTEST_ASSERT("Start point of segment", s1.Belongs(p1));
	SUBTEST_ASSERT("End point of segment", s1.Belongs(p1 + v1));

	SUBTEST_ASSERT("Random point of line", l1.Belongs(p2));
	SUBTEST_ASSERT("Random point of ray", r1.Belongs(p2));
	SUBTEST_ASSERT("Random point of segment", s1.Belongs(p2));

	SUBTEST_ASSERT("Random point not on line", !l1.Belongs(p2 + v2));
	SUBTEST_ASSERT("Random point not on ray", !r1.Belongs(p2 + v2));
	SUBTEST_ASSERT("Random point not on segment", !s1.Belongs(p2 + v2));

	SUBTEST_ASSERT("Negative parameter for ray", !r1.Belongs(p1 - v1));
	SUBTEST_ASSERT("Negative parameter for segment", !s1.Belongs(p1 - v1));
	SUBTEST_ASSERT(">1 parameter for segment", !s1.Belongs(p1 + 2 * v1));

	TEST("Intersections");
	p1 = geomlib::Point<double>(0, 0, 0); p2 = geomlib::Point<double>(3, 2, 0);
	v1 = geomlib::Vector<double>(2, 1, 1); v2 = geomlib::Vector<double>(-2, -2, 2);
	l1 = geomlib::Line<double>(p1, v1); l2 = geomlib::Line<double>(p2, v2);
	r1 = geomlib::Ray<double>(p1, v1); r2 = geomlib::Ray<double>(p2, v2);
	s1 = geomlib::Segment<double>(p1, v1); s2 = geomlib::Segment<double>(p2, v2);

	SUBTEST_ASSERT("Random LL intersection", l1.Intersects(l2));
	SUBTEST_ASSERT("Random LR intersection", l1.Intersects(r2));
	SUBTEST_ASSERT("Random LS intersection", l1.Intersects(s2));
	SUBTEST_ASSERT("Random RR intersection", r1.Intersects(r2));
	SUBTEST_ASSERT("Random RS intersection", r1.Intersects(s2));
	SUBTEST_ASSERT("Random SS intersection", s1.Intersects(s2));

	l2 = geomlib::Line<double>(p1, v2);
	r2 = geomlib::Ray<double>(p1, v2);
	s2 = geomlib::Segment<double>(p1, v2);
	SUBTEST_ASSERT("Same start point LL intersection", l1.Intersects(l2));
	SUBTEST_ASSERT("Same start point LR intersection", l1.Intersects(r2));
	SUBTEST_ASSERT("Same start point LS intersection", l1.Intersects(s2));
	SUBTEST_ASSERT("Same start point RR intersection", r1.Intersects(r2));
	SUBTEST_ASSERT("Same start point RS intersection", r1.Intersects(s2));
	SUBTEST_ASSERT("Same start point SS intersection", s1.Intersects(s2));

	l2 = geomlib::Line<double>(p2, v2);
	r2 = geomlib::Ray<double>(p2, v2.Opposite());
	s2 = geomlib::Segment<double>(p2, v2.Opposite());
	SUBTEST_ASSERT("Intersection with negative parameter for ray", !l1.Intersects(r2));
	SUBTEST_ASSERT("Intersection with negative parameter for segment", !l1.Intersects(s2));
	s2 = geomlib::Segment<double>(p2, 0.4 * v2);
	SUBTEST_ASSERT("Intersection with >1 parameter for segment", !l1.Intersects(s2));

	p1 = geomlib::Point<double>(0, 0, 0); p2 = geomlib::Point<double>(2, 3, 3);
	v1 = geomlib::Vector<double>(4, 2, 0); v2 = geomlib::Vector<double>(2, -2, 0);
	l1 = geomlib::Line<double>(p1, v1); l2 = geomlib::Line<double>(p2, v2);
	r1 = geomlib::Ray<double>(p1, v1); r2 = geomlib::Ray<double>(p2, v2);
	s1 = geomlib::Segment<double>(p1, v1); s2 = geomlib::Segment<double>(p2, v2);
	SUBTEST_ASSERT("Crossing LL", !l1.Intersects(l2));
	SUBTEST_ASSERT("Crossing LR", !l1.Intersects(r2));
	SUBTEST_ASSERT("Crossing LS", !l1.Intersects(s2));
	SUBTEST_ASSERT("Crossing RR", !r1.Intersects(r2));
	SUBTEST_ASSERT("Crossing RS", !r1.Intersects(s2));
	SUBTEST_ASSERT("Crossing SS", !s1.Intersects(s2));

	p1 = geomlib::Point<double>(0, 0, 0); p2 = geomlib::Point<double>(6, -4, 2);
	v1 = geomlib::Vector<double>(-3, 2, -1); v2 = geomlib::Vector<double>(3, -2, 1);
	r1 = geomlib::Ray<double>(p1, v1); r2 = geomlib::Ray<double>(p2, v2);
	SUBTEST_ASSERT("No intersection of opposite rays", !r1.Intersects(r2));
	r2 = geomlib::Ray<double>(p1, v2);
	SUBTEST_ASSERT("Same start point of opposite rays", r1.Intersects(r2));
	r1 = geomlib::Ray<double>(p2, v1); r2 = geomlib::Ray<double>(p1, v2);
	SUBTEST_ASSERT("Intersection of opposite rays", r1.Intersects(r2));

	r2 = geomlib::Ray<double>(p1, v1);
	SUBTEST_ASSERT("Same direction collinear rays 1", r1.Intersects(r2));
	SUBTEST_ASSERT("Same direction collinear rays 2", r2.Intersects(r1));

	l1 = geomlib::Line<double>(p1, v1); l2 = geomlib::Line<double>(p2, v2);
	SUBTEST_ASSERT("Same lines 1", l1.Intersects(l2));
	l2 = geomlib::Line<double>(p2, v1);
	SUBTEST_ASSERT("Same lines 2", l1.Intersects(l2));

	s1 = geomlib::Segment<double>(p1, -2 * v1); s2 = geomlib::Segment<double>(p2, -2 * v2);
	SUBTEST_ASSERT("Same segments", s1.Intersects(s2));


	TEST("Points of intersection");

	SUBTEST_ASSERT("Same segments", s1.Belongs(s1.FindIntersection(s2)) && s2.Belongs(s1.FindIntersection(s2)));
	l1 = geomlib::Line<double>(p1, v1); l2 = geomlib::Line<double>(p2, v2);
	SUBTEST_ASSERT("Same lines 1", l1.Belongs(l1.FindIntersection(l2)) && l2.Belongs(l1.FindIntersection(l2)));
	l2 = geomlib::Line<double>(p2, v1);
	SUBTEST_ASSERT("Same lines 2", l1.Belongs(l1.FindIntersection(l2)) && l2.Belongs(l1.FindIntersection(l2)));
	r1 = geomlib::Ray<double>(p1, v1); r2 = geomlib::Ray<double>(p1, v2);
	SUBTEST_ASSERT("Same start point of opposite rays", r1.Belongs(r1.FindIntersection(r2)) && r2.Belongs(r1.FindIntersection(r2)));
	r1 = geomlib::Ray<double>(p2, v1); r2 = geomlib::Ray<double>(p1, v2);
	SUBTEST_ASSERT("Intersection of opposite rays", r1.Belongs(r1.FindIntersection(r2)) && r2.Belongs(r1.FindIntersection(r2)));
	r2 = geomlib::Ray<double>(p1, v1);
	SUBTEST_ASSERT("Same direction collinear rays 1", r1.Belongs(r1.FindIntersection(r2)) && r2.Belongs(r1.FindIntersection(r2)));
	SUBTEST_ASSERT("Same direction collinear rays 2", r1.Belongs(r1.FindIntersection(r2)) && r2.Belongs(r1.FindIntersection(r2)));

	p1 = geomlib::Point<double>(0, 0, 0); p2 = geomlib::Point<double>(3, 2, 0);
	v1 = geomlib::Vector<double>(2, 1, 1); v2 = geomlib::Vector<double>(-2, -2, 2);
	l1 = geomlib::Line<double>(p1, v1); l2 = geomlib::Line<double>(p2, v2);
	r1 = geomlib::Ray<double>(p1, v1); r2 = geomlib::Ray<double>(p2, v2);
	s1 = geomlib::Segment<double>(p1, v1); s2 = geomlib::Segment<double>(p2, v2);
	SUBTEST_ASSERT("Random LL intersection", l1.Belongs(l1.FindIntersection(l2)) && l2.Belongs(l1.FindIntersection(l2)));
	SUBTEST_ASSERT("Random LR intersection", l1.Belongs(l1.FindIntersection(r2)) && r2.Belongs(l1.FindIntersection(r2)));
	SUBTEST_ASSERT("Random LS intersection", l1.Belongs(l1.FindIntersection(s2)) && s2.Belongs(l1.FindIntersection(s2)));
	SUBTEST_ASSERT("Random RR intersection", r1.Belongs(r1.FindIntersection(r2)) && r2.Belongs(r1.FindIntersection(r2)));
	SUBTEST_ASSERT("Random RS intersection", r1.Belongs(r1.FindIntersection(s2)) && s2.Belongs(r1.FindIntersection(s2)));
	SUBTEST_ASSERT("Random SS intersection", s1.Belongs(s1.FindIntersection(s2)) && s2.Belongs(s1.FindIntersection(s2)));

	l2 = geomlib::Line<double>(p1, v2);
	r2 = geomlib::Ray<double>(p1, v2);
	s2 = geomlib::Segment<double>(p1, v2);
	SUBTEST_ASSERT("Same start point LL intersection", l1.Belongs(l1.FindIntersection(l2)) && l2.Belongs(l1.FindIntersection(l2)));
	SUBTEST_ASSERT("Same start point LR intersection", l1.Belongs(l1.FindIntersection(r2)) && r2.Belongs(l1.FindIntersection(r2)));
	SUBTEST_ASSERT("Same start point LS intersection", l1.Belongs(l1.FindIntersection(s2)) && s2.Belongs(l1.FindIntersection(s2)));
	SUBTEST_ASSERT("Same start point RR intersection", r1.Belongs(r1.FindIntersection(r2)) && r2.Belongs(r1.FindIntersection(r2)));
	SUBTEST_ASSERT("Same start point RS intersection", r1.Belongs(r1.FindIntersection(s2)) && s2.Belongs(r1.FindIntersection(s2)));
	SUBTEST_ASSERT("Same start point SS intersection", s1.Belongs(s1.FindIntersection(s2)) && s2.Belongs(s1.FindIntersection(s2)));

	TESTING_SECTION_CLOSE;
}
