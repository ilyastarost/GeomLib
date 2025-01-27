#pragma once
#include "Generic.h"
#include <type_traits>
#include <cmath>

namespace geomlib
{
	template <typename T, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	class Vector : public Coordinates<T>
	{
	public:
		Vector() : Coordinates<T>() {};
		Vector(T xx, T yy) : Coordinates<T>(xx, yy) {};
		Vector(T xx, T yy, T zz) : Coordinates<T>(xx, yy, zz) {};
		Vector(T* begin) : Coordinates<T>(begin) {};
		bool operator== (const Vector<T>& rhs) const
		{
			return IsEqual(rhs);
		}
		bool operator!= (const Vector<T>& rhs) const
		{
			return !IsEqual(rhs);
		}
		Vector<T>& operator*= (double mul) const {
			this->SetX(this->X() * mul);
			this->SetY(this->Y() * mul);
			this->SetZ(this->Z() * mul);
		}
		bool IsEqual(const Vector<T>& vec, T epsPow2 = Epsilon::EpsPow2()) const
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
			return std::acos(vec.DotProduct(*this) / Length() / vec.Length());
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
		bool IsOrthogonal(const Vector<T>& vec) const
		{
			return DotProduct(vec) <= Epsilon::EpsPow2();
		}
		bool IsParallel(const Vector<T>& vec) const
		{
			return CrossProduct(vec).LengthPow2() <= Epsilon::EpsPow2();
		}
		~Vector() {};
	};

	template <typename T, typename N, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0, 
									  typename std::enable_if<(std::is_arithmetic<N>()), int>::type = 0>
	Vector<T> operator* (const Vector<T>& vec, N mul) {
		return Vector<T>(vec.X() * mul, vec.Y() * mul, vec.Z() * mul);
	}

	template <typename T, typename N, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0, 
									  typename std::enable_if<(std::is_arithmetic<N>()), int>::type = 0>
		Vector<T> operator* (N mul, const Vector<T>& vec) {
		return Vector<T>(vec.X() * mul, vec.Y() * mul, vec.Z() * mul);
	}
}