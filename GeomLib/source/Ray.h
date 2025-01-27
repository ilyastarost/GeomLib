#pragma once
#include "Line.h"

namespace geomlib
{
	template <typename T, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	class Ray : public Line<T>
	{
	public:
		Ray() : Line<T>() {};
		Ray(Point<T> pt, Vector<T> vec) : Line<T>(pt, vec) {};
		bool operator== (const Ray<T>& rhs) const
		{
			return this->Start().IsEqual(rhs.Start()) && this->Direction().IsEqual(rhs.Direction());
		}
		Point<T> FindNearestPointToThis(const Point<T>& pt) const override
		{
			T param = GetParameter(pt);
			if (param < 0) param = 0;
			return Point<T>(this->m_ptStart.X() + param * this->m_vecDirection.X(),
				this->m_ptStart.Y() + param * this->m_vecDirection.Y(),
				this->m_ptStart.Z() + param * this->m_vecDirection.Z());
		}
	};
}