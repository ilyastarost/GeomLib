#pragma once
#include "Coordinates.h"
#include <type_traits>
#include <concepts>
#include <cmath>

namespace geomlib 
{
	template <typename T> requires std::is_floating_point<T>::value
	class Point : public Coordinates<T>
	{
	public:
		Point() : Coordinates<T>() {};
		Point(T xx, T yy) : Coordinates<T>(xx, yy) {};
		Point(T xx, T yy, T zz) : Coordinates<T>(xx, yy, zz) {};
		Point(T* begin) : Coordinates<T>(begin) {};
		bool operator== (const Point<T>& rhs) const
		{
			return IsEqual(*this, rhs);
		}
		bool operator!= (const Point<T>& rhs) const
		{
			return !IsEqual(*this, rhs);
		}
		bool IsEqual(const Point<T>& pt, const T& epsPow2 = Epsilon.EpsPow2()) const
		{
			return DistancePow2(pt) <= epsPow2;
		}
		T Distance(const Point<T>& vec) const
		{
			return std::sqrt(DistancePow2(vec));
		}
		T DistancePow2(const Point<T>& vec) const
		{
			return (this->X() - vec.X()) * (this->X() - vec.X()) + (this->Y() - vec.Y()) * (this->Y() - vec.Y()) + (this->Z() - vec.Z()) * (this->Z() - vec.Z());
		}
		~Point() {};
	};
}