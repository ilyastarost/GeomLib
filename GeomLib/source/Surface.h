#pragma once
#include "Generic.h"
#include "Segment.h"
#include "Line.h"
#include "Ray.h"

namespace geomlib
{
	template <typename T, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	class Surface
	{
	protected:
		Point<T> m_ptStart;
		Vector<T> m_vecBaseX, m_vecBaseY;
	public:
		Surface() = default;
		Surface(const Point<T>& pt, const Vector<T>& x, const Vector<T>& y) : m_ptStart(pt), m_vecBaseX(x), m_vecBaseY(y) {};
		inline Point<T> Start() const { return m_ptStart; }
		inline Vector<T> VectorX() const { return m_vecBaseX; }
		inline Vector<T> VectorY() const { return m_vecBaseY; }
		inline void SetStart(const Point<T>& pt) { m_ptStart = pt; }
		inline void SetVectorX(const Vector<T>& vec) { m_vecBaseX = vec; }
		inline void SetVectorY(const Vector<T>& vec) { m_vecBaseY = vec; }
		
		template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_base_of<Line<T>, S<T>>()), int>::type = 0>
		bool Intersects(const S<T>& lin) const
		{

		}

		bool Belongs(const Point<T>& pt) const
		{
			return pt.DistancePow2(ProjectionOf(pt)) <= Epsilon::EpsPow2();
		}

		template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_base_of<Line<T>, S<T>>()), int>::type = 0>
		bool Belongs(const S<T>& lin) const
		{

		}

		virtual Point<T> ProjectionOf(const Point<T>& pt) const = 0;

	};
}