#pragma once
#include <type_traits>
#include <concepts>
#include <cmath>

namespace geomlib 
{
	class Epsilon {
	private:
		static double eps;
		static double epsPow2;
	public:
		static void SetEpsilon(const double& val)
		{
			eps = val;
			epsPow2 = eps * eps;
		}
		static double Eps()
		{
			return eps;
		}
		static double EpsPow2()
		{
			return epsPow2;
		}
	};
	

	template <typename T> requires std::is_floating_point<T>::value
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
		Coordinates(T xx, T yy) : m_dblX(xx), m_dblY(yy), m_dblZ(0) {};
		Coordinates(T xx, T yy, T zz) : m_dblX(xx), m_dblY(yy), m_dblZ(zz) {};
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
			return std::sqrt((this->X() - vec.X()) * (this->X() - vec.X()) + (this->Y() - vec.Y()) * (this->Y() - vec.Y()) + (this->Z() - vec.Z()) * (this->Z() - vec.Z()));
		}
		T DistancePow2(const Point<T>& vec) const
		{
			return (this->X() - vec.X()) * (this->X() - vec.X()) + (this->Y() - vec.Y()) * (this->Y() - vec.Y()) + (this->Z() - vec.Z()) * (this->Z() - vec.Z());
		}
		~Point() {};
	};

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
			return this->X() * vec.X() + this->Y() * vec.Y();
		}

		Vector<T> CrossProduct(const Vector<T>& vec)
		{
			return Vector<T>(
				 this->Y() * vec.Z() - this->Z() * vec.Y(),
				-this->X() * vec.Z() + this->Z() * vec.X(),
				 this->X() * vec.Y() - this->Y() * vec.X()
				);
		}
		~Vector() {};
	};

	template <typename T> requires std::is_floating_point<T>::value
	Vector<T> operator* (double mul, const Vector<T>& vec) {
		return Vector<T>(vec.X() * mul, vec.Y() * mul, vec.Z() * mul);
	}

	template <typename T, typename S>
	concept type_check = std::is_floating_point<T>::value && std::derived_from<S, Coordinates<T>>;

	template <typename T, typename S> requires type_check<T, S>
	S operator+ (const S& lhs, const Vector<T>& rhs) 
	{
		return S(lhs.X() + rhs.X(), lhs.Y() + rhs.Y());
	}

	template <typename T, typename S> requires type_check<T, S>
	S operator- (const S& lhs, const Vector<T>& rhs) 
	{
		return S(lhs.X() + rhs.X(), lhs.Y() + rhs.Y());
	}

	template <typename T, typename S> requires type_check<T, S>
	S& operator+= (S& lhs, const Vector<T>& rhs) 
	{
		lhs.SetX(lhs.X() + rhs.X());
		lhs.SetY(lhs.Y() + rhs.Y());
		return lhs;
	}

	template <typename T, typename S> requires type_check<T, S>
	S& operator-= (S& lhs, const Vector<T>& rhs) 
	{
		lhs.SetX(lhs.X() - rhs.X());
		lhs.SetY(lhs.Y() - rhs.Y());
		return lhs;
	}

}