#pragma once
#include "Coordinates.h"
#include "Epsilon.h"
#include "Vector.h"
#include "Point.h"
#include <type_traits>
#include <concepts>

namespace geomlib
{
	template <typename T, template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	S<T> operator+ (const S<T>& lhs, const Vector<T>& rhs)
	{
		return S<T>(lhs.X() + rhs.X(), lhs.Y() + rhs.Y(), lhs.Z() + rhs.Z());
	}

	template <typename T, template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	S<T> operator- (const S<T>& lhs, const Vector<T>& rhs)
	{
		return S<T>(lhs.X() - rhs.X(), lhs.Y() - rhs.Y(), lhs.Z() + rhs.Z());
	}

	template <typename T, template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	S<T>& operator+= (S<T>& lhs, const Vector<T>& rhs)
	{
		lhs.SetX(lhs.X() + rhs.X());
		lhs.SetY(lhs.Y() + rhs.Y());
		lhs.SetZ(lhs.Z() + rhs.Z());
		return lhs;
	}

	template <typename T, template<typename, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0> typename S, typename std::enable_if<(std::is_floating_point<T>()), int>::type = 0>
	S<T>& operator-= (S<T>& lhs, const Vector<T>& rhs)
	{
		lhs.SetX(lhs.X() - rhs.X());
		lhs.SetY(lhs.Y() - rhs.Y());
		lhs.SetZ(lhs.Z() - rhs.Z());
		return lhs;
	}
}