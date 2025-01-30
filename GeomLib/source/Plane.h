#pragma once
#include "Surface.h"
#include <utility>

namespace geomlib
{
	FLOATING(T)
	class Plane : public Surface<T> {
	protected:
		Vector<T> m_vecNormal;
		static Point<T> FindPointOfIntersection(const Plane<T>& pl, const Line<T>& lin) {
			if (pl.Belongs(lin.Start())) return lin.Start();
			Point<T> tmp = pl.ProjectionOf(lin.Start());
			Vector<T> norm = tmp - lin.Start();
			T param = norm.LengthPow2() / lin.Direction().DotProduct(norm);
			return lin.Start() + param * lin.Direction();
		}

	public:
		Plane() : Surface<T>() {};
		Plane(const Point<T>&pt, const Vector<T>& norm) 
		{
			this->m_ptStart = pt;
			m_vecNormal = norm;
		};

		inline const Vector<T>& Normal() const { return m_vecNormal; }
		inline void SetNormal(const Vector<T>& norm) { m_vecNormal = norm; }

		Point<T> ProjectionOf(const Point<T>& pt) const 
		{
			T param = (this->Start() - pt).DotProduct(Normal()) / Normal().LengthPow2();
			return pt + param * Normal();
		}

		Vector<T> ProjectionOf(const Vector<T>& pt) const
		{
			T paramb = (this->Start() - Point<T>(0, 0, 0)).DotProduct(Normal()) / Normal().LengthPow2();
			T parame = (this->Start() - pt).DotProduct(Normal()) / Normal().LengthPow2();
			return pt + parame * Normal() - paramb * Normal();
		}

		DERIVED_FROM_LINE(S)
		S<T> ProjectionOf(const S<T>& pt) const
		{
			return S<T>(ProjectionOf(pt.Start()), ProjectionOf(pt.Direction()));
		}

		DERIVED_FROM_LINE(S)
		std::vector<Point<T>> FindIntersections(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			if (abs(this->Normal().DotProduct(lin.Direction())) <= eps) return std::vector<Point<T>>();
			Point<T> pt = FindPointOfIntersection(*this, Line<T>(lin.Start(), lin.Direction()));
			if (lin.Belongs(pt)) return { pt };
			return std::vector<Point<T>>();
		}

		bool Belongs(const Point<T>& pt, T eps = Epsilon::Eps()) const
		{
			Vector<T> tmp = pt - this->Start();
			return (abs(tmp.DotProduct(this->Normal())) <= eps);
		}

		bool GetParameters(const Point<T>& pt, T& param1, T& param2, T eps = Epsilon::Eps()) const {
			Vector<T> base1 = Normal().GetOrthogonal();
			Vector<T> base2 = Normal().CrossProduct(base1);
			if (!base1.CrossProduct(base2).IsParallel(this->Normal())) return false;
			if (!Belongs(pt)) return false;
			if (abs(base1.X() * base2.Y() - base1.Y() * base2.X()) > eps)
			{
				if (abs(base1.X()) > eps)
				{
					param2 = (base1.X() * (pt.Y() - this->Start().Y()) - base1.Y() * (pt.X() - this->Start().X())) / (base1.X() * base2.Y() - base1.Y() * base2.X());
					param1 = (pt.X() - this->Start().X() - base2.X() * param2) / base1.X();
				}
				else 
				{
					param2 = (base1.Y() * (pt.X() - this->Start().X()) - base1.X() * (pt.Y() - this->Start().Y())) / (base1.Y() * base2.X() - base1.X() * base2.Y());
					param1 = (pt.Y() - this->Start().Y() - base2.Y() * param2) / base1.Y();
				}
			}
			else if (abs(base1.Y() * base2.Z() - base1.Z() * base2.Y()) > eps)
			{
				if (abs(base1.Y()) > eps)
				{
					param2 = (base1.Y() * (pt.Z() - this->Start().Z()) - base1.Z() * (pt.Y() - this->Start().Y())) / (base1.Y() * base2.Z() - base1.Z() * base2.Y());
					param1 = (pt.Y() - this->Start().Y() - base2.Y() * param2) / base1.Y();
				}
				else
				{
					param2 = (base1.Z() * (pt.Y() - this->Start().Y()) - base1.Y() * (pt.Z() - this->Start().Z())) / (base1.Z() * base2.Y() - base1.Y() * base2.Z());
					param1 = (pt.Z() - this->Start().Z() - base2.Z() * param2) / base1.Z();
				}
			}
			else {
				if (abs(base1.Z()) > eps)
				{
					param2 = (base1.Z() * (pt.X() - this->Start().X()) - base1.X() * (pt.Z() - this->Start().Z())) / (base1.Z() * base2.X() - base1.X() * base2.Z());
					param1 = (pt.Z() - this->Start().Z() - base2.Z() * param2) / base1.Z();
				}
				else
				{
					param2 = (base1.X() * (pt.Z() - this->Start().Z()) - base1.Z() * (pt.X() - this->Start().X())) / (base1.X() * base2.Z() - base1.Z() * base2.X());
					param1 = (pt.X() - this->Start().X() - base2.X() * param2) / base1.X();
				}
			}
			return true;
		}

		Point<T> GetPointByParameters(T param1, T param2) const
		{
			Vector<T> base1 = Normal().GetOrthogonal();
			Vector<T> base2 = Normal().CrossProduct(base1);
			return this->Start() + base1 * param1 + base2 * param2;
		}

		bool GetNormalIn(const Point<T>& pt, Vector<T>& norm) const
		{
			if (!Belongs(pt)) return false;
			norm = m_vecNormal;
			return true;
		}

	};
}