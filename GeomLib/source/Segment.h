#pragma once
#include "Line.h"

namespace geomlib
{
	template <typename T, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	class Segment : public Line<T>
	{
	public:
		Segment() : Line<T>() {};
		Segment(Point<T> pt, Vector<T> vec) : Line<T>(pt, vec) {};
		Segment(Point<T> pt1, Point<T> pt2)
		{
			this->m_ptStart = pt1;
			this->m_vecDirection = Vector<T>(pt2.X() - pt1.X(), pt2.Y() - pt1.Y(), pt2.Z() - pt1.Z());
		}
		bool operator== (const Segment<T>& rhs) const
		{
			return this->Start().IsEqual(rhs.Start()) && this->Direction().IsEqual(rhs.Direction());
		}
		inline Point<T> End() {
			return this->Start() + this->Direction();
		}
		Point<T> FindNearestPointToThis(const Point<T>& pt) const override
		{
			T param = this->GetParameter(pt);
			if (param < 0) param = 0;
			else if (param > 1) param = 1;
			return Point<T>(this->m_ptStart.X() + param * this->m_vecDirection.X(),
				this->m_ptStart.Y() + param * this->m_vecDirection.Y(),
				this->m_ptStart.Z() + param * this->m_vecDirection.Z());
		}
	};
}