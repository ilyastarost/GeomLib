#pragma once
#include "Line.h"

namespace geomlib
{
	FLOATING(T)
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
			T param = this->GetParameter(pt);
			if (param < 0) param = 0;
			return this->m_ptStart + param * this->m_vecDirection;
		}
	};
}