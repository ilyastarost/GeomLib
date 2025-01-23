#pragma once
#include "Coordinates.h"
#include "Epsilon.h"
#include "Vector.h"
#include "Point.h"
#include <type_traits>
#include <concepts>

namespace geomlib
{
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