#include "Coordinates.h"

namespace geomlib
{
	template <typename T> requires std::is_floating_point<T>::value
	Coordinates<T>::Coordinates()
	{
		m_dblX = 0;
		m_dblY = 0;
		m_dblZ = 0;
	};

	template <typename T> requires std::is_floating_point<T>::value
	Coordinates<T>::Coordinates(T xx, T yy) : m_dblX(xx), m_dblY(yy), m_dblZ(0) {};

	template <typename T> requires std::is_floating_point<T>::value
	Coordinates<T>::Coordinates(T xx, T yy, T zz) : m_dblX(xx), m_dblY(yy), m_dblZ(zz) {};

	template <typename T> requires std::is_floating_point<T>::value
	Coordinates<T>::Coordinates(T* begin)
	{
		T* tmp = begin;
		m_dblX = *tmp;
		tmp = std::next(tmp);
		m_dblY = *tmp;
		tmp = std::next(tmp);
		m_dblZ = *tmp;
	}

	template <typename T> requires std::is_floating_point<T>::value
	inline T geomlib::Coordinates<T>::X() const { return m_dblX; }

	template <typename T> requires std::is_floating_point<T>::value
	inline T geomlib::Coordinates<T>::Y() const { return m_dblY; }

	template <typename T> requires std::is_floating_point<T>::value
	inline T geomlib::Coordinates<T>::Z() const { return m_dblZ; }

	template <typename T> requires std::is_floating_point<T>::value
	inline void geomlib::Coordinates<T>::SetX(T xx) { m_dblX = xx; }

	template <typename T> requires std::is_floating_point<T>::value
	inline void geomlib::Coordinates<T>::SetY(T yy) { m_dblY = yy; }

	template <typename T> requires std::is_floating_point<T>::value
	inline void geomlib::Coordinates<T>::SetZ(T zz) { m_dblZ = zz; }

	template <typename T> requires std::is_floating_point<T>::value
	bool geomlib::Coordinates<T>::operator== (const Coordinates& rhs) const
	{
		return IsEqual(*this, rhs);
	}
	template <typename T> requires std::is_floating_point<T>::value
	Coordinates<T>::~Coordinates() = default;
}