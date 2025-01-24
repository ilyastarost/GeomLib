#pragma once
#include "Generic.h"

namespace geomlib
{
	template <typename T> requires std::is_floating_point<T>::value
		class Line
	{
	protected:
		Point<T> m_ptStart;
		Vector<T> m_vecDirection;
		T GetParameter(const Point<T>& pt) const
		{
			return ((pt.X() - m_ptStart.X()) * m_vecDirection.X() +
				(pt.Y() - m_ptStart.Y()) * m_vecDirection.Y() +
				(pt.Z() - m_ptStart.Z()) * m_vecDirection.Z()) /
				(m_vecDirection.X() * m_vecDirection.X() +
					m_vecDirection.Y() * m_vecDirection.Y() +
					m_vecDirection.Z() * m_vecDirection.Z());
		}
		static Point<T> FindPointOfIntersection(const Line<T>& lin1, const Line<T>& lin2) {
			T m = (lin1.Direction().X() * (lin1.Start().Y() - lin2.Start().Y()) +
				lin1.Direction().Y() * (lin2.Start().X() - lin1.Start().X())) /
				(lin1.Direction().X() * lin2.Direction().Y() - lin2.Direction().X() * lin1.Direction().Y());
			return Point<T>(lin2.Start() + m * lin2.Direction());
		}
	public:
		Line() = default;
		Line(const Point<T>& pt, const Vector<T>& vec) : m_ptStart(pt), m_vecDirection(vec) {};
		inline Point<T> Start() const { return m_ptStart; }
		inline Vector<T> Direction() const { return m_vecDirection; }
		inline void SetStart(const Point<T>& pt) { m_ptStart = pt; }
		inline void SetDirection(const Vector<T>& vec) { m_vecDirection = vec; }

		Point<T> FindNearestPointToLine(const Point<T>& pt) const {
			T param = this->GetParameter(pt);
			return Point<T>(this->m_ptStart.X() + param * this->m_vecDirection.X(),
				this->m_ptStart.Y() + param * this->m_vecDirection.Y(),
				this->m_ptStart.Z() + param * this->m_vecDirection.Z());
		}
		Point<T> FindNearestPointToThis(const Point<T>& pt) const {
			return FindNearestPointToLine(pt);
		}
		T DistanceToLine(const Point<T>& pt) const
		{
			return pt.Distance(FindNearestPointToLine(pt));
		}
		T DistanceToLinePow2(const Point<T>& pt) const
		{
			return pt.DistancePow2(FindNearestPointToLine(pt));
		}
		T DistanceToThis(const Point<T>& pt) const
		{
			return pt.Distance(FindNearestPointToThis(pt));
		}
		T DistanceToThisPow2(const Point<T>& pt) const
		{
			return pt.DistancePow2(FindNearestPointToThis(pt));
		}
		template <typename T, template<typename> typename S>
		bool IsCollinear(const S<T>& lin) const
		{
			T dist = DistancePow2(lin.Start());
			T ang = this->m_vecDirection.Angle(lin.Direction());
			return (dist <= Epsilon::EpsPow2() && ang <= Epsilon::Eps());
		}

		template <typename T, template<typename> typename S>
		bool IsOrthogonal(const S<T>& lin) const
		{
			return m_vecDirection.DotProduct(lin.Direction()) <= Epsilon::EpsPow2();
		}

		template <typename T, template<typename> typename S>
		bool IsParallel(const S<T>& lin) const
		{
			return m_vecDirection.CrossProduct(lin.Direction()) <= Epsilon::EpsPow2();
		}

		template <typename T, template<typename> typename S>
		bool Intersects(const S<T>& lin) const
		{
			Vector<T> tmp(m_ptStart.X() - lin.Start().X(),
				m_ptStart.Y() - lin.Start().Y(),
				m_ptStart.Z() - lin.Start().Z());
			if (tmp.DotProduct(lin.Direction().CrossProduct(m_vecDirection)) <= Epsilon::Eps())
			{
				Point<T> ans = FindPointOfIntersection(Line<T>(m_ptStart, m_vecDirection), Line<T>(lin.Start(), lin.Direction()));
				return (Belongs(ans) && lin.Belongs(ans));
			}
			return false;
		}

		template <typename T, template<typename> typename S>
		Point<T> FindIntersection(const S<T>& lin) const
		{
			if (Intersects(lin)) return FindPointOfIntersection(Linear<T>(m_ptStart, m_vecDirection), Linear<T>(lin.Start(), lin.Direction()));
		}

		bool Belongs(const Point<T>& pt) const
		{
			return DistanceToLinePow2(pt) <= Epsilon::EpsPow2();
		}
	};

	template <typename T> requires std::is_floating_point<T>::value
		class Ray : public Line<T>
	{
	public:
		Ray() : Linear<T>() {};
		Ray(Point<T> pt, Vector<T> vec) : Linear<T>(pt, vec) {};
		Point<T> FindNearestPointToThis(const Point<T>& pt) const override
		{
			T param = GetParameter(pt);
			if (param < 0) param = 0;
			return Point<T>(this->m_ptStart.X() + param * this->m_vecDirection.X(),
				this->m_ptStart.Y() + param * this->m_vecDirection.Y(),
				this->m_ptStart.Z() + param * this->m_vecDirection.Z());
		}
	};

	template <typename T> requires std::is_floating_point<T>::value
		class Segment : public Line<T>
	{
	public:
		Segment() : Linear<T>() {};
		Segment(Point<T> pt, Vector<T> vec) : Linear<T>(pt, vec) {};
		Segment(Point<T> pt1, Point<T> pt2)
		{
			this->m_ptStart = pt1;
			this->m_vecDirection = Vector<T>(pt2.X() - pt1.X(), pt2.Y() - pt1.Y(), pt2.Z() - pt1.Z());
		}
		inline Point<T> End() {
			return this->Start() + this->Direction();
		}
		Point<T> FindNearestPoint(const Point<T>& pt) const override
		{
			T param = GetParameter(pt);
			if (param < 0) param = 0;
			else if (param > 1) param = 1;
			return Point<T>(this->m_ptStart.X() + param * this->m_vecDirection.X(),
				this->m_ptStart.Y() + param * this->m_vecDirection.Y(),
				this->m_ptStart.Z() + param * this->m_vecDirection.Z());
		}
	};
}