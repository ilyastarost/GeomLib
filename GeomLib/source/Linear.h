#pragma once
#include "Vector.h"
#include "Point.h"

namespace geomlib
{
	template <typename T, typename S>
	concept linear_check = std::is_floating_point<T>::value && std::derived_from<S, Linear<T>>;

	template <typename T> requires std::is_floating_point<T>::value
		class Linear
	{
	protected:
		Point<T> m_ptStart;
		Vector<T> m_vecDirection;
		T GetCoefficient(const Point<T>& pt) const
		{
			return ((pt.X() - m_ptStart.X()) * m_vecDirection.X() +
					(pt.Y() - m_ptStart.Y()) * m_vecDirection.Y() +
					(pt.Z() - m_ptStart.Z()) * m_vecDirection.Z()) /
					(m_vecDirection.X() * m_vecDirection.X() +
					 m_vecDirection.Y() * m_vecDirection.Y() +
					 m_vecDirection.Z() * m_vecDirection.Z());
		}
	public:
		Linear() = default;
		Linear(const Point<T>& pt, const Vector<T>& vec) : m_ptStart(pt), m_vecDirection(vec) {};
		inline Point<T> Start() const { return m_ptStart; }
		inline Point<T> Direction() const { return m_vecDirection; }
		inline void SetStart(const Point<T>& pt) { m_ptStart = pt; }
		inline void SetDirection(const Vector<T>& vec) { m_vecDirection = vec; }

		virtual Point<T> FindNearestPoint(const Point<T>& pt) const = 0;
		T Distance(const Point<T>& pt) const
		{
			return m_ptStart.Distance(FindNearestPoint(pt));
		}
		T DistancePow2(const Point<T>& pt) const
		{
			return m_ptStart.DistancePow2(FindNearestPoint(pt));
		}

		template <typename T, typename S> requires linear_check<T, S>
		bool IsCollinear(const S&) const {};

		template <typename T, typename S> requires linear_check<T, S>
		bool IsOrthogonal(const S& lin) const
		{
			return m_vecDirection.DotProduct(lin.Direction()) <= Epsilon.EpsPow2();
		}

		template <typename T, typename S> requires linear_check<T, S>
		bool IsParallel(const S& lin) const
		{
			return m_vecDirection.CrossProduct(lin.Direction()) <= Epsilon.EpsPow2();
		}

		template <typename T, typename S> requires linear_check<T, S>
		bool Intersects(const S&) const {};

		template <typename T, typename S> requires linear_check<T, S>
		Point<T> FindIntersection(const S&) const {};

		bool Belongs(const Point<T>& pt) const
		{
			return DistancePow2(pt) <= Epsilon.EpsPow2();
		}
	};

	template <typename T> requires std::is_floating_point<T>::value
		class Line : public Linear<T>
	{
	public:
		Line() : Linear<T>() {};
		Line(Point<T> pt, Vector<T> vec) : Linear<T>(pt, vec) {};
		Point<T> FindNearestPoint(const Point<T>& pt) const override
		{
			T coef = GetCoefficient(pt);
			return Point<T>(this->m_ptStart.X() + coef * this->m_vecDirection.X(),
							this->m_ptStart.Y() + coef * this->m_vecDirection.Y(),
							this->m_ptStart.Z() + coef * this->m_vecDirection.Z());
		}
		template <typename T, typename S> requires linear_check<T, S>
		bool IsCollinear(const S& lin) const
		{
			T dist = DistancePow2(lin.Start());
			T ang = this->m_vecDirection.Angle(lin.Direction());
			return (dist <= Epsilon.EpsPow2() && ang <= Epsilon.Eps());
		}
	};

	template <typename T> requires std::is_floating_point<T>::value
		class Ray : public Linear<T>
	{
	public:
		Ray() : Linear<T>() {};
		Ray(Point<T> pt, Vector<T> vec) : Linear<T>(pt, vec) {};
		Point<T> FindNearestPoint(const Point<T>& pt) const override
		{
			T coef = GetCoefficient(pt);
			if (coef < 0) coef = 0;
			return Point<T>(this->m_ptStart.X() + coef * this->m_vecDirection.X(),
							this->m_ptStart.Y() + coef * this->m_vecDirection.Y(),
							this->m_ptStart.Z() + coef * this->m_vecDirection.Z());
		}
		template <typename T, typename S> requires linear_check<T, S>
		bool IsCollinear(const S& lin) const
		{
			Line<T> tmp(this->m_ptStart, this->m_vecDirection);
			return tmp.IsCollinear(lin);
		}
	};

	template <typename T> requires std::is_floating_point<T>::value
		class Segment : public Linear<T>
	{
	public:
		Segment() : Linear<T>() {};
		Segment(Point<T> pt, Vector<T> vec) : Linear<T>(pt, vec) {};
		Segment(Point<T> pt1, Point<T> pt2)
		{
			this->m_ptStart = pt1;
			this->m_vecDirection = Vector<T>(pt2.X() - pt1.X(), pt2.Y() - pt1.Y(), pt2.Z() - pt1.Z());
		}
		Point<T> FindNearestPoint(const Point<T>& pt) const override
		{
			T coef = GetCoefficient(pt);
			if (coef < 0) coef = 0;
			else if (coef > 1) coef = 1;
			return Point<T>(this->m_ptStart.X() + coef * this->m_vecDirection.X(),
							this->m_ptStart.Y() + coef * this->m_vecDirection.Y(),
							this->m_ptStart.Z() + coef * this->m_vecDirection.Z());
		}
		template <typename T, typename S> requires linear_check<T, S>
		bool IsCollinear(const S& lin) const
		{
			Line<T> tmp(this->m_ptStart, this->m_vecDirection);
			return tmp.IsCollinear(lin);
		}
	};
}