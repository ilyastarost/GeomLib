#pragma once
#include "Surface.h"
#include <utility>

namespace geomlib
{
	FLOATING(T)
	class Plane : public Surface<T> {
	protected:
		static Point<T> FindPointOfIntersection(const Plane<T>& pl, const Line<T>& lin) {
			Point<T> tmp = pl.ProjectionOf(lin.Start());
			Vector<T> norm(tmp.X() - lin.Start().X(), tmp.Y() - lin.Start().Y(), tmp.Z() - lin.Start().Z());
			T param = (norm.X() * norm.X() + norm.Y() * norm.X() + norm.Z() * norm.Z()) /
				(lin.Direction().X() * norm.X() + lin.Direction().Y() * norm.Y() + lin.Direction().Z() * norm.Z());
			return Point<T>(lin.Start().X() + param * lin.Direction().X(), lin.Start().Y() + param * lin.Direction().Y(), lin.Start().Z() + param * lin.Direction().Z());
		}

	public:
		Plane() : Surface<T>() {};
		Plane(const Point<T>&pt, const Vector<T>& norm) : Surface<T>(pt, norm) {};

		inline Vector<T> Normal() const { return this->m_vecBase; }
		inline void SetNormal(const Vector<T>& norm) { this->m_vecBase = norm; }

		std::pair<Vector<T>, Vector<T>> GetBase() const {
			std::pair<Vector<T>, Vector<T>> ans;
			ans.first = this->m_vecBase.GetOrthogonal();
			ans.second = ans.first.CrossProduct(this->m_vecBase);
			return ans;
		}

		Point<T> ProjectionOf(const Point<T>& pt) const 
		{
			T param =  ((this->Start().X() - pt.X()) * Normal().X() +
						(this->Start().Y() - pt.Y()) * Normal().Y() +
						(this->Start().Z() - pt.Z()) * Normal().Z()) /
						(Normal().X() * Normal().X() + Normal().Y() * Normal().Y() + Normal().Z() * Normal().Z());
			return Point<T>(pt.X() + param * Normal().X(), pt.Y() + param * Normal().Y(), pt.Z() + param * Normal().Z());
		}

		DERIVED_FROM_LINE(S)
		bool Intersects(const S<T>& lin) const
		{
			if (AreEqual(0, this->Normal().DotProduct(lin.Direction()))) return false;
			Point<T> pt = FindPointOfIntersection(*this, Line<T>(lin.Start(), lin.Direction()));
			return lin.Belongs(pt);
		}

		DERIVED_FROM_LINE(S)
		Point<T> FindIntersection(const S<T>& lin) const
		{
			if (Intersects(lin)) return FindPointOfIntersection(*this, Line<T>(lin.Start(), lin.Direction()));
			return Point<T>(NAN, NAN, NAN);
		}

		bool Belongs(const Point<T>& pt) const
		{
			Vector<T> tmp(pt.X() - this->Start().X(), pt.Y() - this->Start().Y(), pt.Z() - this->Start().Z());
			return (tmp.DotProduct(this->Normal()) <= Epsilon::Eps());
		}

		std::pair<T, T> GetParameters(const Point<T>& pt, const Vector<T>& base1, const Vector<T>& base2) {
			if (!base1.CrossProduct(base2).IsParallel(this->Normal)) return { NAN, NAN };
			if (!Belongs(pt)) return { NAN, NAN };
			if (base1.IsParallel(base2)) return { NAN, NAN };
			if (abs(base1.X() * base2.Y() - base1.Y() * base2.X()) > Epsilon::Eps() && abs(base1.X()) > Epsilon::Eps())
			{
				T s = (base1.X() * (pt.Y() - this->Start().Y()) - base1.Y() * (this->Start().X() - pt.X())) / (base1.X() * base2.Y() - base1.Y() * base2.X());
				T t = (pt.X() - this->Start().X() - base2.X() * s) / base1.X();
				return { t, s };
			}
			else if (abs(base1.Y() * base2.Z() - base1.Z() * base2.Y()) > Epsilon::Eps() && abs(base1.Y()) > Epsilon::Eps())
			{
				T s = (base1.Y() * (pt.Z() - this->Start().Z()) - base1.Z() * (this->Start().Y() - pt.Y())) / (base1.Y() * base2.Z() - base1.Z() * base2.Y());
				T t = (pt.Y() - this->Start().Y() - base2.Y() * s) / base1.Y();
				return { t, s };
			}
			else {
				T s = (base1.Z() * (pt.X() - this->Start().X()) - base1.X() * (this->Start().Z() - pt.Z())) / (base1.Z() * base2.X() - base1.X() * base2.Z());
				T t = (pt.Z() - this->Start().Z() - base2.Z() * s) / base1.Z();
				return { t, s };
			}
		}

	};
}