#include "source/Cylinder.h"
#include "source/Testing.h"
#include "source/Segment.h"
#include "source/Matrix.h"
#include "source/Plane.h"
#include "source/Line.h"
#include "source/Ray.h"
#include <iostream>
#include <fstream>

using namespace geomlib;

int main()
{
	TESTING_SECTION_OPEN;

	TEST("Multiplication");
	Point<double> p1(1, 1, 1), p2(2, 3, -1);
	Vector<double> v1(3, -1, 1), v2(1, 2, -1), zero(0, 0, 0);
	Line<double> l1(p1, v1), l2(p2, v1);

	SUBTEST_EQ("Dot parallel", v1.DotProduct(2 * v1), 22);
	SUBTEST_EQ("Cross parallel", v1.CrossProduct(2 * v1), Vector<double>(0, 0, 0));
	
	SUBTEST_EQ("Dot orthogonal", v1.DotProduct(v2), 0);
	SUBTEST_EQ("Cross orthogonal", v1.CrossProduct(v2), Vector<double>(-1, 4, 7));

	SUBTEST_EQ("Dot zero", v1.DotProduct(zero), 0);
	SUBTEST_EQ("Cross zero", v1.CrossProduct(zero), zero);


	TEST("Belonging");
	p1 = Point<double>(-1, 3, 2); p2 = Point<double>(0, 2.5, 2.5);
	v1 = Vector<double>(2, -1, 1);
	l1 = Line<double>(p1, v1);
	Ray<double> r1(p1, v1), r2;
	Segment<double> s1(p1, v1), s2;

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
	p1 = Point<double>(0, 0, 0); p2 = Point<double>(3, 2, 0);
	v1 = Vector<double>(2, 1, 1); v2 = Vector<double>(-2, -2, 2);
	l1 = Line<double>(p1, v1); l2 = Line<double>(p2, v2);
	r1 = Ray<double>(p1, v1); r2 = Ray<double>(p2, v2);
	s1 = Segment<double>(p1, v1); s2 = Segment<double>(p2, v2);

	SUBTEST_ASSERT("Random LL intersection", l1.Intersects(l2));
	SUBTEST_ASSERT("Random LR intersection", l1.Intersects(r2));
	SUBTEST_ASSERT("Random LS intersection", l1.Intersects(s2));
	SUBTEST_ASSERT("Random RR intersection", r1.Intersects(r2));
	SUBTEST_ASSERT("Random RS intersection", r1.Intersects(s2));
	SUBTEST_ASSERT("Random SS intersection", s1.Intersects(s2));

	l2 = Line<double>(p1, v2);
	r2 = Ray<double>(p1, v2);
	s2 = Segment<double>(p1, v2);
	SUBTEST_ASSERT("Same start point LL intersection", l1.Intersects(l2));
	SUBTEST_ASSERT("Same start point LR intersection", l1.Intersects(r2));
	SUBTEST_ASSERT("Same start point LS intersection", l1.Intersects(s2));
	SUBTEST_ASSERT("Same start point RR intersection", r1.Intersects(r2));
	SUBTEST_ASSERT("Same start point RS intersection", r1.Intersects(s2));
	SUBTEST_ASSERT("Same start point SS intersection", s1.Intersects(s2));

	l2 = Line<double>(p2, v2);
	r2 = Ray<double>(p2, v2.Opposite());
	s2 = Segment<double>(p2, v2.Opposite());
	SUBTEST_ASSERT("Intersection with negative parameter for ray", !l1.Intersects(r2));
	SUBTEST_ASSERT("Intersection with negative parameter for segment", !l1.Intersects(s2));
	s2 = Segment<double>(p2, 0.4 * v2);
	SUBTEST_ASSERT("Intersection with >1 parameter for segment", !l1.Intersects(s2));

	p1 = Point<double>(0, 0, 0); p2 = Point<double>(2, 3, 3);
	v1 = Vector<double>(4, 2, 0); v2 = Vector<double>(2, -2, 0);
	l1 = Line<double>(p1, v1); l2 = Line<double>(p2, v2);
	r1 = Ray<double>(p1, v1); r2 = Ray<double>(p2, v2);
	s1 = Segment<double>(p1, v1); s2 = Segment<double>(p2, v2);
	SUBTEST_ASSERT("Crossing LL", !l1.Intersects(l2));
	SUBTEST_ASSERT("Crossing LR", !l1.Intersects(r2));
	SUBTEST_ASSERT("Crossing LS", !l1.Intersects(s2));
	SUBTEST_ASSERT("Crossing RR", !r1.Intersects(r2));
	SUBTEST_ASSERT("Crossing RS", !r1.Intersects(s2));
	SUBTEST_ASSERT("Crossing SS", !s1.Intersects(s2));

	p1 = Point<double>(0, 0, 0); p2 = Point<double>(6, -4, 2);
	v1 = Vector<double>(-3, 2, -1); v2 = Vector<double>(3, -2, 1);
	r1 = Ray<double>(p1, v1); r2 = Ray<double>(p2, v2);
	SUBTEST_ASSERT("No intersection of opposite rays", !r1.Intersects(r2));
	r2 = Ray<double>(p1, v2);
	SUBTEST_ASSERT("Same start point of opposite rays", r1.Intersects(r2));
	r1 = Ray<double>(p2, v1); r2 = Ray<double>(p1, v2);
	SUBTEST_ASSERT("Intersection of opposite rays", r1.Intersects(r2));

	r2 = Ray<double>(p1, v1);
	SUBTEST_ASSERT("Same direction collinear rays 1", r1.Intersects(r2));
	SUBTEST_ASSERT("Same direction collinear rays 2", r2.Intersects(r1));

	l1 = Line<double>(p1, v1); l2 = Line<double>(p2, v2);
	SUBTEST_ASSERT("Same lines 1", l1.Intersects(l2));
	l2 = Line<double>(p2, v1);
	SUBTEST_ASSERT("Same lines 2", l1.Intersects(l2));

	s1 = Segment<double>(p1, -2 * v1); s2 = Segment<double>(p2, -2 * v2);
	SUBTEST_ASSERT("Same segments", s1.Intersects(s2));


	TEST("Points of intersection");

	SUBTEST_ASSERT("Same segments", s1.Belongs(s1.FindIntersections(s2)[0]) && s2.Belongs(s1.FindIntersections(s2)[0]));
	l1 = Line<double>(p1, v1); l2 = Line<double>(p2, v2);
	SUBTEST_ASSERT("Same lines 1", l1.Belongs(l1.FindIntersections(l2)[0]) && l2.Belongs(l1.FindIntersections(l2)[0]));
	l2 = Line<double>(p2, v1);
	SUBTEST_ASSERT("Same lines 2", l1.Belongs(l1.FindIntersections(l2)[0]) && l2.Belongs(l1.FindIntersections(l2)[0]));
	r1 = Ray<double>(p1, v1); r2 = Ray<double>(p1, v2);
	SUBTEST_ASSERT("Same start point of opposite rays", r1.Belongs(r1.FindIntersections(r2)[0]) && r2.Belongs(r1.FindIntersections(r2)[0]));
	r1 = Ray<double>(p2, v1); r2 = Ray<double>(p1, v2);
	SUBTEST_ASSERT("Intersection of opposite rays", r1.Belongs(r1.FindIntersections(r2)[0]) && r2.Belongs(r1.FindIntersections(r2)[0]));
	r2 = Ray<double>(p1, v1);
	SUBTEST_ASSERT("Same direction collinear rays 1", r1.Belongs(r1.FindIntersections(r2)[0]) && r2.Belongs(r1.FindIntersections(r2)[0]));
	SUBTEST_ASSERT("Same direction collinear rays 2", r1.Belongs(r1.FindIntersections(r2)[0]) && r2.Belongs(r1.FindIntersections(r2)[0]));

	p1 = Point<double>(0, 0, 0); p2 = Point<double>(3, 2, 0);
	v1 = Vector<double>(2, 1, 1); v2 = Vector<double>(-2, -2, 2);
	l1 = Line<double>(p1, v1); l2 = Line<double>(p2, v2);
	r1 = Ray<double>(p1, v1); r2 = Ray<double>(p2, v2);
	s1 = Segment<double>(p1, v1); s2 = Segment<double>(p2, v2);
	SUBTEST_ASSERT("Random LL intersection", l1.Belongs(l1.FindIntersections(l2)[0]) && l2.Belongs(l1.FindIntersections(l2)[0]));
	SUBTEST_ASSERT("Random LR intersection", l1.Belongs(l1.FindIntersections(r2)[0]) && r2.Belongs(l1.FindIntersections(r2)[0]));
	SUBTEST_ASSERT("Random LS intersection", l1.Belongs(l1.FindIntersections(s2)[0]) && s2.Belongs(l1.FindIntersections(s2)[0]));
	SUBTEST_ASSERT("Random RR intersection", r1.Belongs(r1.FindIntersections(r2)[0]) && r2.Belongs(r1.FindIntersections(r2)[0]));
	SUBTEST_ASSERT("Random RS intersection", r1.Belongs(r1.FindIntersections(s2)[0]) && s2.Belongs(r1.FindIntersections(s2)[0]));
	SUBTEST_ASSERT("Random SS intersection", s1.Belongs(s1.FindIntersections(s2)[0]) && s2.Belongs(s1.FindIntersections(s2)[0]));

	l2 = Line<double>(p1, v2);
	r2 = Ray<double>(p1, v2);
	s2 = Segment<double>(p1, v2);
	SUBTEST_ASSERT("Same start point LL intersection", l1.Belongs(l1.FindIntersections(l2)[0]) && l2.Belongs(l1.FindIntersections(l2)[0]));
	SUBTEST_ASSERT("Same start point LR intersection", l1.Belongs(l1.FindIntersections(r2)[0]) && r2.Belongs(l1.FindIntersections(r2)[0]));
	SUBTEST_ASSERT("Same start point LS intersection", l1.Belongs(l1.FindIntersections(s2)[0]) && s2.Belongs(l1.FindIntersections(s2)[0]));
	SUBTEST_ASSERT("Same start point RR intersection", r1.Belongs(r1.FindIntersections(r2)[0]) && r2.Belongs(r1.FindIntersections(r2)[0]));
	SUBTEST_ASSERT("Same start point RS intersection", r1.Belongs(r1.FindIntersections(s2)[0]) && s2.Belongs(r1.FindIntersections(s2)[0]));
	SUBTEST_ASSERT("Same start point SS intersection", s1.Belongs(s1.FindIntersections(s2)[0]) && s2.Belongs(s1.FindIntersections(s2)[0]));


	TEST("Surfaces");

	p1 = Point<double>(0, 0, 0); p2 = Point<double>(-4, -2, 2);
	v1 = Vector<double>(0, 0, 1); v2 = Vector<double>(0, 3, -3);
	l1 = Line<double>(p2, v2);
	r1 = Ray<double>(p2, v2);
	s1 = Segment<double>(p2, v2);
	Cylinder<double> c1(p1, v1, 2);
	Plane<double> pl(p1, v1);
	double u, v;
	SUBTEST_ASSERT("Plane parameters 1", !pl.GetParameters(Point<double>(4, 1, -1), u, v));
	SUBTEST_ASSERT("Plane parameters 2", pl.GetParameters(Point<double>(4, 1, 0), u, v));
	SUBTEST_ASSERT("Plane parameters 3", pl.GetPointByParameters(u, v) == Point<double>(4, 1, 0));
	SUBTEST_ASSERT("Cylinder parameters 1", c1.GetParameters(Point<double>(2, 0, 4), u, v));
	SUBTEST_ASSERT("Cylinder parameters 2", u == 4 && v == acos(-1) / 2);
	auto q = c1.GetPointByParameters(u, v);
	SUBTEST_ASSERT("Cylinder parameters 3", q == Point<double>(2, 0, 4));

	SUBTEST_ASSERT("Plane intersects line", pl.FindIntersections(l1)[0] == Point<double>(-4, 0, 0));
	SUBTEST_ASSERT("Plane intersects ray", pl.FindIntersections(r1)[0] == Point<double>(-4, 0, 0));
	SUBTEST_ASSERT("Plane intersects segment", pl.FindIntersections(s1)[0] == Point<double>(-4, 0, 0));

	r1 = Ray<double>(p2, v2.Opposite());
	s1 = Segment<double>(p2, v2.Opposite());
	SUBTEST_ASSERT("Plane doesn't intersect ray", pl.FindIntersections(r1).size() == 0);
	SUBTEST_ASSERT("Plane doesn't intersect segment", pl.FindIntersections(s1).size() == 0);


	TEST("Matrices");

	Matrix<double> one = Matrix<double>::GetIdentity(), tmp;
	Matrix<double> tr = Matrix<double>::TranslationInit(v2);
	Matrix<double> rotX = Matrix<double>::RotationAroundXInit(acos(-1) / 4);
	Matrix<double> rot = Matrix<double>::RotationInit(Vector<double>(0, 2, 2), acos(-1) / 2);
	Matrix<double> sc = Matrix<double>::ScalingInit(Vector<double>(3, 1, -1));
	Matrix<double> coordTrue = Matrix<double>::ToCoordinatesInit(Vector<double>(1, 0, 0), Vector<double>(1, 1, 0), Vector<double>(1, 1, 1));
	Matrix<double> coordFalse = Matrix<double>::ToCoordinatesInit(Vector<double>(1, 0, 0), Vector<double>(2, 0, 1), Vector<double>(1, 0, 3));

	SUBTEST_EQ("Translate point", Point<double>(1, -1, 2) * tr, Point<double>(1, 2, -1));
	SUBTEST_EQ("Rotate point around X", Point<double>(0, 1, 1) * rotX, Point<double>(0, 0, sqrt(2)));
	SUBTEST_EQ("Rotate point on axis around X", Point<double>(2, 0, 0) * rotX, Point<double>(2, 0, 0));
	SUBTEST_EQ("Rotate point around vector", Point<double>(1, 0, 0) * rot, Point<double>(0, sqrt(2) / 2, -sqrt(2) / 2));
	SUBTEST_EQ("Rotate point on axis around vector", Point<double>(0, -3, -3) * rot, Point<double>(0, -3, -3));
	SUBTEST_EQ("Scale point", Point<double>(1, 2, 3) * sc, Point<double>(3, 2, -3));
	SUBTEST_EQ("Scale zero", Point<double>(0, 0, 0) * sc, Point<double>(0, 0, 0));
	SUBTEST_EQ("Point to new coordinates 1", Point<double>(4, 2, 1) * coordTrue, Point<double>(2, 1, 1));
	SUBTEST_EQ("Point to new coordinates 2", Point<double>(-2, -2, -2) * coordTrue, Point<double>(0, 0, -2));
	SUBTEST_EQ("Zero to new coordinates", Point<double>(0, 0, 0)* coordTrue, Point<double>(0, 0, 0));

	SUBTEST_EQ("Rotation around X + new coordinates", Point<double>(0, 1, 1) * rotX * coordTrue, Point<double>(0, -sqrt(2), sqrt(2)));
	SUBTEST_EQ("Rotation around vector + new coordinates", Point<double>(1, 0, 0) * rot * coordTrue, Point<double>(-sqrt(2) / 2, sqrt(2), -sqrt(2) / 2));
	SUBTEST_EQ("Translation + new coordinates", Point<double>(1, -1, 2) * tr * coordTrue, Point<double>(-1, 3, -1));
	tmp = coordTrue.InvertedCopy();
	SUBTEST_EQ("Invertion", coordTrue * tmp, one);
	//SUBTEST_ASSERT("Impossible invertion", !coordFalse.Invertion(tmp));

	TESTING_SECTION_CLOSE;

	std::cout << p1.ToString() << std::endl;
	std::cout << v1.ToString() << std::endl;
	std::cout << l1.ToString() << std::endl;
	std::cout << r1.ToString() << std::endl;
	std::cout << s1.ToString() << std::endl;
	std::cout << rot.ToString() << std::endl;
	std::cout << pl.ToString() << std::endl;
	std::cout << c1.ToString() << std::endl;
	std::ofstream bout("out.bin", std::fstream::binary);
	p2.Serialize(bout);
	v2.Serialize(bout);
	l2.Serialize(bout);
	r2.Serialize(bout);
	s2.Serialize(bout);
	rot.Serialize(bout);
	pl.Serialize(bout);
	c1.Serialize(bout);
	bout.close();
	std::ifstream bin("out.bin", std::fstream::binary);
	p1.Deserialize(bin);
	v1.Deserialize(bin);
	l1.Deserialize(bin);
	r1.Deserialize(bin);
	s1.Deserialize(bin);
	tmp.Deserialize(bin);
	int y = 0;
}