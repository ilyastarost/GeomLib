#pragma once
#include <type_traits>
#include <iterator>

#define FLOATING(T) template <typename T, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
#define FLOATING(V) template <typename V, typename std::enable_if<(std::is_floating_point<V>()), int>::type = 0>

namespace geomlib
{
	FLOATING(T)
	class Coordinates
	{
	protected:
		T m_dblX, m_dblY, m_dblZ;
	public:
		Coordinates()
		{
			m_dblX = 0;
			m_dblY = 0;
			m_dblZ = 0;
		};
		Coordinates(T xx, T yy, T zz = 0) : m_dblX(xx), m_dblY(yy), m_dblZ(zz) {};
		Coordinates(T* begin)
		{
			T* tmp = begin;
			m_dblX = *tmp;
			tmp = std::next(tmp);
			m_dblY = *tmp;
			tmp = std::next(tmp);
			m_dblZ = *tmp;
		}
		inline T X() const { return m_dblX; }
		inline T Y() const { return m_dblY; }
		inline T Z() const { return m_dblZ; }
		inline void SetX(T xx) { m_dblX = xx; }
		inline void SetY(T yy) { m_dblY = yy; }
		inline void SetZ(T zz) { m_dblZ = zz; }
		bool operator== (const Coordinates& rhs) const
		{
			return IsEqual(*this, rhs);
		}
		~Coordinates() {};
	};
}