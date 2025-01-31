#pragma once
#include "Generic.h"
#include "Segment.h"
#include "Matrix.h"
#include "Line.h"
#include "Ray.h"
#include <vector>

namespace geomlib
{
	FLOATING(T)
	class Surface
	{
	protected:
		Point<T> m_ptStart;
	public:
		Surface() = default;
		Surface(const Point<T>& pt, const Vector<T>& a) : m_ptStart(pt) {};
		inline Point<T> Start() const { return m_ptStart; }
		inline void SetStart(const Point<T>& pt) { m_ptStart = pt; }
		

		bool Belongs(const Point<T>& pt, T epsPow2 = Epsilon::EpsPow2()) const
		{
			return pt.DistancePow2(ProjectionOf(pt)) <= epsPow2;
		}

		virtual Point<T> ProjectionOf(const Point<T>& pt) const = 0;

		virtual bool GetParameters(const Point<T>& pt, T& param1, T& param2, T eps = Epsilon::Eps()) const = 0;

		virtual Point<T> GetPointByParameters(T param1, T param2) const = 0;

		virtual bool GetNormalIn(const Point<T>& pt, Vector<T>& norm) const = 0;

	};
}