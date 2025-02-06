#pragma once
#include "Coordinates.h"
#include <type_traits>
#include <cmath>

namespace geomlib 
{
	FLOATING(T)
	class Point : public Coordinates<T>
	{
	public:
		Point() : Coordinates<T>() {};
		Point(T xx, T yy, T zz = 0) : Coordinates<T>(xx, yy, zz) {};
		Point(T* begin) : Coordinates<T>(begin) {};
		bool operator== (const Point<T>& rhs) const
		{
			return IsEqual(rhs);
		}
		bool operator!= (const Point<T>& rhs) const
		{
			return !IsEqual(rhs);
		}
		bool IsEqual(const Point<T>& pt, T epsPow2 = Epsilon::EpsPow2()) const
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

		std::string ToString() const override
		{
			std::stringstream out;
			out << "Point (" << typeid(T).name() << ") with:" << std::endl;
			out << Coordinates<T>::ToString();
			return out.str();
		}
	};
}