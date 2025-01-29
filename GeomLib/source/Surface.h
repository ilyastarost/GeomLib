#pragma once
#include "Generic.h"
#include "Segment.h"
#include "Line.h"
#include "Ray.h"

namespace geomlib
{
	FLOATING(T)
	class Surface
	{
	protected:
		Point<T> m_ptStart;
		Vector<T> m_vecBase;
	public:
		Surface() = default;
		Surface(const Point<T>& pt, const Vector<T>& a) : m_ptStart(pt), m_vecBase(a) {};
		inline Point<T> Start() const { return m_ptStart; }
		inline void SetStart(const Point<T>& pt) { m_ptStart = pt; }
		
		DERIVED_FROM_LINE(S)
		bool Intersects(const S<T>& lin) const;

		bool Belongs(const Point<T>& pt) const
		{
			return pt.DistancePow2(ProjectionOf(pt)) <= Epsilon::EpsPow2();
		}

		virtual Point<T> ProjectionOf(const Point<T>& pt) const = 0;

	};
}