#pragma once
#include "Generic.h"

namespace geomlib
{
	template <typename T, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
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
		template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S>
		bool IsCollinear(const S<T>& lin) const
		{
			T dist = DistanceToLinePow2(lin.Start());
			T ang = this->m_vecDirection.Angle(lin.Direction());
			return (dist <= Epsilon::EpsPow2() && ang <= Epsilon::Eps());
		}

		template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S>
		bool IsOrthogonal(const S<T>& lin) const
		{
			return m_vecDirection.IsOrthogonal(lin.Direction());
		}

		template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S>
		bool IsParallel(const S<T>& lin) const
		{
			return m_vecDirection.IsParallel(lin.Direction());
		}

		template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S>
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

		template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S>
		Point<T> FindIntersection(const S<T>& lin) const
		{
			if (Intersects(lin)) return FindPointOfIntersection(Line<T>(m_ptStart, m_vecDirection), Line<T>(lin.Start(), lin.Direction()));
			return Point<T>(NAN, NAN);
		}

		bool Belongs(const Point<T>& pt) const
		{
			return DistanceToLinePow2(pt) <= Epsilon::EpsPow2();
		}
	};
}