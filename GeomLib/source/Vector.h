#pragma once
#include "Coordinates.h"
#include "Epsilon.h"
#include <type_traits>
#include <concepts>
#include <cmath>

namespace geomlib
{
	template <typename T> requires std::is_floating_point<T>::value
	class Vector : public Coordinates<T>
	{
	public:
		Vector() : Coordinates<T>() {};
		Vector(T xx, T yy) : Coordinates<T>(xx, yy) {};
		Vector(T xx, T yy, T zz) : Coordinates<T>(xx, yy, zz) {};
		Vector(T* begin) : Coordinates<T>(begin) {};
		bool operator== (const Vector<T>& rhs) const
		{
			return IsEqual(*this, rhs);
		}
		bool operator!= (const Vector<T>& rhs) const
		{
			return !IsEqual(*this, rhs);
		}
		Vector<T> operator* (double mul) const {
			return Vector<T>(this->X() * mul, this->Y() * mul, this->Z() * mul);
		}
		Vector<T>& operator*= (double mul) const {
			this->SetX(this->X() * mul);
			this->SetY(this->Y() * mul);
			this->SetZ(this->Z() * mul);
		}
		bool IsEqual(const Vector<T>& vec, const T& epsPow2 = Epsilon.EpsPow2())
		{
			return (*this - vec).LengthPow2() <= epsPow2;
		}
		T Length() const
		{
			return std::sqrt(this->X() * this->X() + this->Y() * this->Y() + this->Z() * this->Z());
		}
		T LengthPow2() const
		{
			return this->X() * this->X() + this->Y() * this->Y() + this->Z() * this->Z();
		}
		void Normalize()
		{
			T len = Length();
			this->SetX(this->X() / len);
			this->SetY(this->Y() / len);
			this->SetZ(this->Z() / len);
		}
		Vector<T> NormalizedCopy() const
		{
			T len = Length();
			return Vector<T>(this->X() / len, this->Y() / len, this->Z() / len);
		}
		T Angle(const Vector<T>& vec) const
		{
			return std::acos(DotProduct(*this, vec) / Length() / vec.Length());
		}
		T DotProduct(const Vector<T>& vec) const
		{
			return this->X() * vec.X() + this->Y() * vec.Y() + this->Z() * vec.Z();
		}
		Vector<T> CrossProduct(const Vector<T>& vec) const
		{
			return Vector<T>(
				this->Y() * vec.Z() - this->Z() * vec.Y(),
				-this->X() * vec.Z() + this->Z() * vec.X(),
				this->X() * vec.Y() - this->Y() * vec.X()
				);
		}
		Vector<T> Opposite() const {
			return Vector<T>(-this->X(), -this->Y(), -this->Z());
		}
		bool IsOpposite(const Vector<T>& vec) const
		{
			return vec == Opposite();
		}
		~Vector() {};
	};

	template <typename T> requires std::is_floating_point<T>::value
		Vector<T> operator* (double mul, const Vector<T>& vec) {
		return Vector<T>(vec.X() * mul, vec.Y() * mul, vec.Z() * mul);
	}
}