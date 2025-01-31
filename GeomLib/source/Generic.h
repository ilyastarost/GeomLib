#pragma once
#include "Epsilon.h"
#include "Vector.h"
#include <type_traits>
#include <cmath>

#define DERIVED_FROM_COORDINATES(S, T) template <typename T, template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_base_of<Coordinates<T>, S<T>>()), int>::type = 0>
#define DERIVED_FROM_COORDINATES_FRIEND(S, T) template <typename T, template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S>
#define DERIVED_FROM_LINE(S) template <template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_base_of<Line<T>, S<T>>()), int>::type = 0>

namespace geomlib
{
	DERIVED_FROM_COORDINATES(S, T)
	S<T> operator+ (const S<T>& lhs, const Vector<T>& rhs)
	{
		return S<T>(lhs.X() + rhs.X(), lhs.Y() + rhs.Y(), lhs.Z() + rhs.Z());
	}

	DERIVED_FROM_COORDINATES(S, T)
	S<T> operator- (const S<T>& lhs, const Vector<T>& rhs)
	{
		return S<T>(lhs.X() - rhs.X(), lhs.Y() - rhs.Y(), lhs.Z() - rhs.Z());
	}

	DERIVED_FROM_COORDINATES(S, T)
	S<T>& operator+= (S<T>& lhs, const Vector<T>& rhs)
	{
		lhs.SetX(lhs.X() + rhs.X());
		lhs.SetY(lhs.Y() + rhs.Y());
		lhs.SetZ(lhs.Z() + rhs.Z());
		return lhs;
	}

	DERIVED_FROM_COORDINATES(S, T)
	S<T>& operator-= (S<T>& lhs, const Vector<T>& rhs)
	{
		lhs.SetX(lhs.X() - rhs.X());
		lhs.SetY(lhs.Y() - rhs.Y());
		lhs.SetZ(lhs.Z() - rhs.Z());
		return lhs;
	}

	template <typename T, typename S, typename std::enable_if<(std::is_arithmetic<T>()), int>::type = 0, typename std::enable_if<(std::is_arithmetic<S>()), int>::type = 0>
	bool AreEqual(T lhs, S rhs, double eps = Epsilon::Eps())
	{
		return abs(lhs - rhs) <= eps;
	}
}