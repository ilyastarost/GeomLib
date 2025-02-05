#pragma once
#include <type_traits>
#include <iterator>
#include <iostream>
#include <sstream>

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

		virtual std::string ToString() const
		{
			std::stringstream out;
			out << "        X = " << m_dblX << std::endl;
			out << "        Y = " << m_dblY << std::endl;
			out << "        Z = " << m_dblZ << std::endl;
			return out.str();
		}

		void Serialize(std::ostream& out) const
		{
			out.write((char*)&m_dblX, sizeof(T));
			out.write((char*)&m_dblY, sizeof(T));
			out.write((char*)&m_dblZ, sizeof(T));
		}

		void Deserialize(std::istream& in)
		{
			in.read((char*)&this->m_dblX, sizeof(T));
			in.read((char*)&this->m_dblY, sizeof(T));
			in.read((char*)&this->m_dblZ, sizeof(T));
		}
	};
}