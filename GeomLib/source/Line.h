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
			return (pt - m_ptStart).DotProduct(m_vecDirection) /
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
			return this->m_ptStart + param * this->m_vecDirection;
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
		bool IsCollinear(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			T dist = DistanceToLinePow2(lin.Start());
			T ang = this->m_vecDirection.Angle(lin.Direction());
			return (dist <= eps * eps && (abs(ang) <= eps || abs(ang - acos(-1)) <= eps));
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
		bool Intersects(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			Vector<T> tmp = m_ptStart - lin.Start();
			if (abs(tmp.DotProduct(lin.Direction().CrossProduct(m_vecDirection))) <= eps)
			{
				if (IsCollinear(lin)) {
					if (this->Belongs(lin.Start()) || lin.Belongs(m_ptStart)) return true;
					return false;
				}
				Point<T> ans = FindPointOfIntersection(AsLine(), lin.AsLine());
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
			if (Intersects(lin)) return FindPointOfIntersection(AsLine(), lin.AsLine());
			return Point<T>(NAN, NAN);
		}

		bool Belongs(const Point<T>& pt) const
		{
			return DistanceToThisPow2(pt) <= Epsilon::EpsPow2();
		}

		Line<T> AsLine() const {
			return Line<T>(m_ptStart, m_vecDirection);
		}
	};
}