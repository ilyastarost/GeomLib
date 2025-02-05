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

		std::string ToString() const
		{
			std::stringstream out;
			out << "Ray with start point: ";
			out << this->m_ptStart.ToString();
			out << "    And direction: ";
			out << this->m_vecDirection.ToString();
			return out.str();
		}
		void Serialize(std::ostream& out) const
		{
			this->m_ptStart.Serialize(out);
			this->m_vecDirection.Serialize(out);
		}
		void Deserialize(std::istream& in)
		{
			this->m_ptStart.Deserialize(in);
			this->m_vecDirection.Deserialize(in);
		}
	};
}