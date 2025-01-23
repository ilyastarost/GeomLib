#pragma once
#include <type_traits>
#include <concepts>
#include <cmath>

namespace geomlib {

	static double epsilon = 1e-6;
	void SetEpsilon(double val) {
		epsilon = val;
	}


	template <typename T> requires std::is_floating_point<T>::value
		class Coordinates2D {
		protected:
			T x, y;
		public:
			Coordinates2D() {
				x = 0;
				y = 0;
			};
			Coordinates2D(T xx, T yy) : x(xx), y(yy) {};
			T X() const {
				return x;
			}
			T Y() const {
				return y;
			}
			void SetX(T xx) {
				x = xx;
			}
			void SetY(T yy) {
				y = yy;
			}
			Coordinates2D& operator= (const Coordinates2D& rhs) {
				x = rhs.x;
				y = rhs.y;
				return *this;
			}
			bool operator== (const Coordinates2D& rhs) const {
				if (this->X() == rhs.X() && this->Y() == rhs.Y()) return true;
				return false;
			}
			~Coordinates2D() {};
	};

	template <typename T> requires std::is_floating_point<T>::value
		class Point2D : public Coordinates2D<T> {
		public:
			Point2D() : Coordinates2D<T>() {};
			Point2D(T xx, T yy) : Coordinates2D<T>(xx, yy) {};
			Point2D& operator= (const Point2D& rhs) {
				this->SetX(rhs.X());
				this->SetY(rhs.Y());
				return *this;
			}
			bool operator== (const Point2D& rhs) const {
				if (this->X() == rhs.X() && this->Y() == rhs.Y()) return true;
				return false;
			}
			bool operator!= (const Point2D& rhs) const {
				return !(rhs == *this);
			}
			T dist_to(const Point2D& rhs) const {
				return std::sqrt((this->X() - rhs.X()) * (this->X() - rhs.X()) + (this->Y() - rhs.Y()) * (this->Y() - rhs.Y()));
			}
			~Point2D() {};
	};

	template <typename T> requires std::is_floating_point<T>::value
		class Vector2D : public Coordinates2D<T> {
		public:
			Vector2D() : Coordinates2D<T>() {};
			Vector2D(T xx, T yy) : Coordinates2D<T>(xx, yy) {};
			Vector2D& operator= (const Vector2D& rhs) {
				this->SetX(rhs.X());
				this->SetY(rhs.Y());
				return *this;
			}
			bool operator== (const Vector2D& rhs) const {
				if (this->X() == rhs.X() && this->Y() == rhs.Y()) return true;
				return false;
			}
			bool operator!= (const Vector2D& rhs) const {
				return !(rhs == *this);
			}
			~Vector2D() {};
	};

	template <typename T, typename S>
	concept type_check = std::is_floating_point<T>::value && std::derived_from<S, Coordinates2D<T>>;

	template <typename T, typename S> requires type_check<T, S>
	S operator+ (const S& lhs, const Vector2D<T>& rhs) {
		return Point2D(lhs.X() + rhs.X(), lhs.Y() + rhs.Y());
	}

	template <typename T, typename S> requires type_check<T, S>
	S operator- (const S& lhs, const Vector2D<T>& rhs) {
		return Point2D(lhs.X() + rhs.X(), lhs.Y() + rhs.Y());
	}

	template <typename T, typename S> requires type_check<T, S>
	S& operator+= (S& lhs, const Vector2D<T>& rhs) {
		lhs.SetX(lhs.X() + rhs.X());
		lhs.SetY(lhs.Y() + rhs.Y());
		return lhs;
	}

	template <typename T, typename S> requires type_check<T, S>
	S& operator-= (S& lhs, const Vector2D<T>& rhs) {
		lhs.SetX(lhs.X() - rhs.X());
		lhs.SetY(lhs.Y() - rhs.Y());
		return lhs;
	}

}