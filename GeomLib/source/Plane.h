#pragma once
#include "Surface.h"

namespace geomlib
{
	template <typename T, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	class Plane : public Surface<T> {
		Plane() : Surface() {};
		Surface(const Point<T>&pt, const Vector<T>&x, const Vector<T>&y) : Plane(pt, x, y) {};
	};
}