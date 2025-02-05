#pragma once
#include "Line.h"

namespace geomlib
{
	FLOATING(T)
	class Segment : public Line<T>
	{
	public:
		Segment() : Line<T>() {};
		Segment(Point<T> pt, Vector<T> vec) : Line<T>(pt, vec) {};
		Segment(Point<T> pt1, Point<T> pt2)
		{
			this->m_ptStart = pt1;
			this->m_vecDirection = pt2 - pt1;
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
			return this->m_ptStart + param * this->m_vecDirection;
		}
		std::string ToString() const
		{
			std::stringstream out;
			out << "Segment with start point: ";
			out << this->m_ptStart.ToString();
			out << "    And end point: ";
			out << (this->m_ptStart + this->m_vecDirection).ToString();
			return out.str();
		}
		void Serialize(std::ostream& out) const
		{
			this->m_ptStart.Serialize(out);
			(this->m_ptStart + this->m_vecDirection).Serialize(out);
		}
		void Deserialize(std::istream& in)
		{
			this->m_ptStart.Deserialize(in);
			Point<T> end;
			end.Deserialize(in);
			this->m_vecDirection = end - this->m_ptStart;
		}
	};
}