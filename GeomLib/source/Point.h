#pragma once
#include <type_traits>
#include <concepts>
#include <cmath>

namespace geomlib 
{
	double epsilon = 1e-6;
	void SetEpsilon(const double& val) 
	{
		epsilon = val;
	}
	double GetEpsilon() 
	{
		return epsilon;
	}

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
		Coordinates(T arr[3])
		{
			m_dblX = arr[0];
			m_dblY = arr[1];
			m_dblZ = arr[2];
		}
		inline T X() const { return m_dblX; }
		inline T Y() const { return m_dblY; }
		inline T Z() const { return m_dblZ; }
		inline void SetX(T xx) { m_dblX = xx; }
		inline void SetY(T yy) { m_dblY = yy; }
		inline void SetZ(T zz) { m_dblZ = zz; }
		bool operator== (const Coordinates& rhs) const 
		{
			return !IsEqual(*this, rhs);
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
		Point(T arr[3]) : Coordinates<T>(arr) {};
		bool operator== (const Point<T>& rhs) const
		{
			return IsEqual(*this, rhs);
		}
		bool operator!= (const Point<T>& rhs) const
		{
			return !IsEqual(*this, rhs);
		}
		T DistanceTo(const Point<T>& rhs) const
		{
			return DistanceBetween(*this, rhs);
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
		Vector(T arr[3]) : Coordinates<T>(arr) {};
		bool operator== (const Vector<T>& rhs) const
		{
			return IsEqual(*this, rhs);
		}
		bool operator!= (const Vector<T>& rhs) const
		{
			return !IsEqual(*this, rhs);
		}
		T Length() const
		{
			return LengthOf(*this);
		}
		void Normalize()
		{
			T len = Length();
			this->SetX(this->X() / len);
			this->SetY(this->Y() / len);
			this->SetZ(this->Z() / len);
		}
		T AngleTo(const Vector<T>& vec) const
		{
			return AngleBetween(*this, vec);
		}
		void CrossMul(const Vector<T>& vec) {
			Vector<T> res = CrossProduct(*this, vec);
			this->SetX(res.X());
			this->SetY(res.Y());
			this->SetZ(res.Z());
		}
		~Vector() {};
	};

	template <typename T> requires std::is_floating_point<T>::value
	bool IsEqual(const Point<T>& lhs, const Point<T>& rhs, const T& eps = epsilon)
	{
		return lhs.DistanceTo(rhs) <= eps;
	}

	template <typename T> requires std::is_floating_point<T>::value
	bool IsEqual(const Vector<T>& lhs, const Vector<T>& rhs, const T& eps = epsilon)
	{
		return (lhs - rhs).Length() <= eps;
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

	template <typename T> requires std::is_floating_point<T>::value
	T DotProduct(const Vector<T>& lhs, const Vector<T>& rhs) 
	{
		return lhs.X() * rhs.X() + lhs.Y() * rhs.Y();
	}

	template <typename T> requires std::is_floating_point<T>::value
	Vector<T> CrossProduct(const Vector<T>& lhs, const Vector<T>& rhs)
	{
		return Vector<T>(
			lhs.Y() * rhs.Z() - lhs.Z() * rhs.Y(),
		   -lhs.X() * rhs.Z() + lhs.Z() * rhs.X(),
			lhs.X() * rhs.Y() - lhs.Y() * rhs.X()
			);
	}

	template <typename T> requires std::is_floating_point<T>::value
	T AngleBetween(const Vector<T>& lhs, const Vector<T>& rhs)
	{
		return std::acos(DotProduct(lhs, rhs) / lhs.Length() / rhs.Length());
	}

	template <typename T> requires std::is_floating_point<T>::value
	T DistanceBetween(const Point<T>& lhs, const Point<T>& rhs)
	{
		return std::sqrt((lhs.X() - rhs.X()) * (lhs.X() - rhs.X()) + (lhs.Y() - rhs.Y()) * (lhs.Y() - rhs.Y()) + (lhs.Z() - rhs.Z()) * (lhs.Z() - rhs.Z()));
	}

	template <typename T> requires std::is_floating_point<T>::value
	Vector<T> Normalized(const Vector<T>& vec) {
		T len = vec.Length();
		return Vector<T>(vec.X() / len, vec.Y() / len, vec.Z() / len);
	}

	template <typename T> requires std::is_floating_point<T>::value
	T LengthOf(const Vector<T>& vec) {
		return std::sqrt(vec.X() * vec.X() + vec.Y() * vec.Y() + vec.Z() * vec.Z());
	}

}