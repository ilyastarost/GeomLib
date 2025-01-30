#pragma once
#include "Generic.h"
#include "Point.h"
#include <type_traits>
#include <cmath>

#define FLOATING_AND_ARITHMETIC(T, N) template <typename T, typename N, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0, typename std::enable_if<(std::is_arithmetic<N>()), int>::type = 0>

namespace geomlib
{
	FLOATING(T)
	class Vector : public Coordinates<T>
	{
	public:
		Vector() : Coordinates<T>() {};
		Vector(T xx, T yy, T zz = 0) : Coordinates<T>(xx, yy, zz) {};
		Vector(T* begin) : Coordinates<T>(begin) {};
		bool operator== (const Vector<T>& rhs) const
		{
			return IsEqual(rhs);
		}
		bool operator!= (const Vector<T>& rhs) const
		{
			return !IsEqual(rhs);
		}
		template <typename N, typename std::enable_if<(std::is_arithmetic<N>()), int>::type = 0>
		Vector<T>& operator*= (N mul) {
			this->SetX(this->X() * mul);
			this->SetY(this->Y() * mul);
			this->SetZ(this->Z() * mul);
			return *this;
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
		Vector<T>& Normalize()
		{
			T len = Length();
			if (len) {
				this->SetX(this->X() / len);
				this->SetY(this->Y() / len);
				this->SetZ(this->Z() / len);
			}
			return *this;
		}
		Vector<T> NormalizedCopy() const
		{
			T len = Length();
			if (len) return Vector<T>(this->X() / len, this->Y() / len, this->Z() / len);
			return *this;
		}
		T Angle(const Vector<T>& vec) const
		{
			if (Length() != 0 && vec.Length() != 0) return std::acos(DotProduct(vec) / Length() / vec.Length());
			return 0;
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
		bool IsOpposite(const Vector<T>& vec, T eps = Epsilon::Eps()) const
		{
			return Angle(vec) <= eps;
		}
		bool IsOrthogonal(const Vector<T>& vec, T epsPow2 = Epsilon::EpsPow2()) const
		{
			return abs(DotProduct(vec)) <= epsPow2;
		}
		bool IsParallel(const Vector<T>& vec, T epsPow2 = Epsilon::EpsPow2()) const
		{
			return CrossProduct(vec).LengthPow2() <= epsPow2;
		}
		Vector<T> GetOrthogonal(T eps = Epsilon::Eps()) const {
			if (abs(this->X()) > eps) {
				return Vector<T>(this->Y(), -this->X(), 0);
			}
			return Vector<T>(0, this->Z(), -this->Y());
		}
		~Vector() {};
	};

	FLOATING_AND_ARITHMETIC(T, N)
	Vector<T> operator* (const Vector<T>& vec, N mul) {
		return Vector<T>(vec.X() * mul, vec.Y() * mul, vec.Z() * mul);
	}

	FLOATING_AND_ARITHMETIC(T, N)
	Vector<T> operator* (N mul, const Vector<T>& vec) {
		return Vector<T>(vec.X() * mul, vec.Y() * mul, vec.Z() * mul);
	}

	FLOATING(T)
	Vector<T> operator- (const Point<T>& lhs, const Point<T>& rhs)
	{
		return Vector<T>(lhs.X() - rhs.X(), lhs.Y() - rhs.Y(), lhs.Z() - rhs.Z());
	}
}