#include "Point.h"

template <typename T> requires std::is_floating_point<T>::value
geomlib::Point<T>::Point() : Coordinates<T>() {};

template <typename T> requires std::is_floating_point<T>::value
geomlib::Point<T>::Point(T xx, T yy) : Coordinates<T>(xx, yy) {};

template <typename T> requires std::is_floating_point<T>::value
geomlib::Point<T>::Point(T xx, T yy, T zz) : Coordinates<T>(xx, yy, zz) {};

template <typename T> requires std::is_floating_point<T>::value
geomlib::Point<T>::Point(T* begin) : Coordinates<T>(begin) {};

template <typename T> requires std::is_floating_point<T>::value
bool geomlib::Point<T>::operator== (const Point<T>& rhs) const
{
	return IsEqual(*this, rhs);
}

template <typename T> requires std::is_floating_point<T>::value
bool geomlib::Point<T>::operator!= (const Point<T>& rhs) const
{
	return !IsEqual(*this, rhs);
}

template <typename T> requires std::is_floating_point<T>::value
bool geomlib::Point<T>::IsEqual(const Point<T>& pt, const T& epsPow2) const
{
	return DistancePow2(pt) <= epsPow2;
}

template <typename T> requires std::is_floating_point<T>::value
T geomlib::Point<T>::Distance(const Point<T>& vec) const
{
	return std::sqrt((this->X() - vec.X()) * (this->X() - vec.X()) + (this->Y() - vec.Y()) * (this->Y() - vec.Y()) + (this->Z() - vec.Z()) * (this->Z() - vec.Z()));
}

template <typename T> requires std::is_floating_point<T>::value
T geomlib::Point<T>::DistancePow2(const Point<T>& vec) const
{
	return (this->X() - vec.X()) * (this->X() - vec.X()) + (this->Y() - vec.Y()) * (this->Y() - vec.Y()) + (this->Z() - vec.Z()) * (this->Z() - vec.Z());
}

template <typename T> requires std::is_floating_point<T>::value
geomlib::Point<T>::~Point() = default;