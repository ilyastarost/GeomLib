#pragma once
#include "Generic.h"

namespace geomlib
{
	FLOATING(T)
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
		bool operator== (const Line<T>& rhs) const
		{
			return Start().IsEqual(rhs.Start()) && Direction().IsEqual(rhs.Direction());
		}
		inline Point<T> Start() const { return m_ptStart; }
		inline Vector<T> Direction() const { return m_vecDirection; }
		inline void SetStart(const Point<T>& pt) { m_ptStart = pt; }
		inline void SetDirection(const Vector<T>& vec) { m_vecDirection = vec; }

		virtual Point<T> FindNearestPointToLine(const Point<T>& pt) const {
			T param = this->GetParameter(pt);
			return Point<T>(this->m_ptStart.X() + param * this->m_vecDirection.X(),
							this->m_ptStart.Y() + param * this->m_vecDirection.Y(),
							this->m_ptStart.Z() + param * this->m_vecDirection.Z());
		}
		virtual Point<T> FindNearestPointToThis(const Point<T>& pt) const {
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
		DERIVED_FROM_LINE(S)
		bool IsCollinear(const S<T>& lin) const
		{
			T dist = DistanceToLinePow2(lin.Start());
			T ang = this->m_vecDirection.Angle(lin.Direction());
			return (dist <= Epsilon::EpsPow2() && (abs(ang) <= Epsilon::Eps() || abs(ang - acos(-1)) <= Epsilon::Eps()));
		}

		DERIVED_FROM_LINE(S)
		bool IsOrthogonal(const S<T>& lin) const
		{
			return m_vecDirection.IsOrthogonal(lin.Direction());
		}

		DERIVED_FROM_LINE(S)
		bool IsParallel(const S<T>& lin) const
		{
			return m_vecDirection.IsParallel(lin.Direction());
		}

		DERIVED_FROM_LINE(S)
		bool Intersects(const S<T>& lin) const
		{
			Vector<T> tmp(m_ptStart.X() - lin.Start().X(),
						  m_ptStart.Y() - lin.Start().Y(),
						  m_ptStart.Z() - lin.Start().Z());
			if (abs(tmp.DotProduct(lin.Direction().CrossProduct(m_vecDirection))) <= Epsilon::Eps())
			{
				if (IsCollinear(lin)) {
					if (this->Belongs(lin.Start()) || lin.Belongs(m_ptStart)) return true;
					return false;
				}
				Point<T> ans = FindPointOfIntersection(Line<T>(m_ptStart, m_vecDirection), Line<T>(lin.Start(), lin.Direction()));
				return (Belongs(ans) && lin.Belongs(ans));
			}
			return false;
		}

		DERIVED_FROM_LINE(S)
		Point<T> FindIntersection(const S<T>& lin) const
		{
			if (IsCollinear(lin)) {
				if (Belongs(lin.Start())) return lin.Start();
				else if (lin.Belongs(m_ptStart)) return m_ptStart;
				return Point<T>(NAN, NAN);
			}
			if (Intersects(lin)) return FindPointOfIntersection(Line<T>(m_ptStart, m_vecDirection), Line<T>(lin.Start(), lin.Direction()));
			return Point<T>(NAN, NAN);
		}

		bool Belongs(const Point<T>& pt) const
		{
			return DistanceToThisPow2(pt) <= Epsilon::EpsPow2();
		}
	};
}