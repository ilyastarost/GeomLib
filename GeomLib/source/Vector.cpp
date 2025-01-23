#include "Vector.h"

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T>::Vector() : Coordinates<T>() {};

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T>::Vector(T xx, T yy) : Coordinates<T>(xx, yy) {};

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T>::Vector(T xx, T yy, T zz) : Coordinates<T>(xx, yy, zz) {};

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T>::Vector(T* begin) : Coordinates<T>(begin) {};

template <typename T> requires std::is_floating_point<T>::value
bool geomlib::Vector<T>::operator== (const Vector<T>& rhs) const
{
	return IsEqual(*this, rhs);
}

template <typename T> requires std::is_floating_point<T>::value
bool geomlib::Vector<T>::operator!= (const Vector<T>& rhs) const
{
	return !IsEqual(*this, rhs);
}

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T> geomlib::Vector<T>::operator* (double mul) const
{
	return Vector<T>(this->X() * mul, this->Y() * mul, this->Z() * mul);
}
template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T>& geomlib::Vector<T>::operator*= (double mul) const
{
	this->SetX(this->X() * mul);
	this->SetY(this->Y() * mul);
	this->SetZ(this->Z() * mul);
}

template <typename T> requires std::is_floating_point<T>::value
bool geomlib::Vector<T>::IsEqual(const Vector<T>& vec, T epsPow2)
{
	return (*this - vec).LengthPow2() <= epsPow2;
}

template <typename T> requires std::is_floating_point<T>::value
T geomlib::Vector<T>::Length() const
{
	return std::sqrt(this->X() * this->X() + this->Y() * this->Y() + this->Z() * this->Z());
}

template <typename T> requires std::is_floating_point<T>::value
T geomlib::Vector<T>::LengthPow2() const
{
	return this->X() * this->X() + this->Y() * this->Y() + this->Z() * this->Z();
}

template <typename T> requires std::is_floating_point<T>::value
void geomlib::Vector<T>::Normalize()
{
	T len = Length();
	this->SetX(this->X() / len);
	this->SetY(this->Y() / len);
	this->SetZ(this->Z() / len);
}

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T> geomlib::Vector<T>::NormalizedCopy() const
{
	T len = Length();
	return Vector<T>(this->X() / len, this->Y() / len, this->Z() / len);
}

template <typename T> requires std::is_floating_point<T>::value
T geomlib::Vector<T>::Angle(const Vector<T>& vec) const
{
	return std::acos(DotProduct(*this, vec) / Length() / vec.Length());
}

template <typename T> requires std::is_floating_point<T>::value
T geomlib::Vector<T>::DotProduct(const Vector<T>& vec) const
{
	return this->X() * vec.X() + this->Y() * vec.Y();
}

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T> geomlib::Vector<T>::CrossProduct(const Vector<T>& vec)
{
	return Vector<T>(
		this->Y() * vec.Z() - this->Z() * vec.Y(),
		-this->X() * vec.Z() + this->Z() * vec.X(),
		this->X() * vec.Y() - this->Y() * vec.X()
		);
}

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T>::~Vector() = default;

template <typename T> requires std::is_floating_point<T>::value
geomlib::Vector<T> geomlib::operator* (double mul, const geomlib::Vector<T>& vec) {
	return Vector<T>(vec.X() * mul, vec.Y() * mul, vec.Z() * mul);
}