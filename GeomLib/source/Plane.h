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
			//params is parameter for start point of vector, parame is parameter for end point
			T params = (this->Start() - Point<T>(0, 0, 0)).DotProduct(Normal()) / Normal().LengthPow2();
			T parame = (this->Start() - pt).DotProduct(Normal()) / Normal().LengthPow2();
			return pt + parame * Normal() - params * Normal();
		}

		DERIVED_FROM_LINE(S)
		S<T> ProjectionOf(const S<T>& pt) const
		{
			return S<T>(ProjectionOf(pt.Start()), ProjectionOf(pt.Direction()));
		}

		DERIVED_FROM_LINE(S)
		std::vector<Point<T>> FindIntersections(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			if (Normal().IsOrthogonal(lin.Direction())) 
				return std::vector<Point<T>>();
			Point<T> pt = FindPointOfIntersection(*this, lin.AsLine());
			if (lin.Belongs(pt)) 
				return { pt };
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
			Matrix<double> tmp = Matrix<double>::ToCoordinatesInit(base1, base2, Normal());
			Point<T> ans = pt * tmp;
			if (abs(ans.Z()) > eps) 
				return false;
			param1 = ans.X();
			param2 = ans.Y();
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

		std::string ToString() const
		{
			std::stringstream out;
			out << "Plane with start point: ";
			out << this->m_ptStart.ToString();
			out << "    And normal: ";
			out << m_vecNormal.ToString();
			return out.str();
		}
		void Serialize(std::ostream& out) const
		{
			this->m_ptStart.Serialize(out);
			m_vecNormal.Serialize(out);
		}

		void Deserialize(std::istream& in)
		{
			this->m_ptStart.Deserialize(in);
			m_vecNormal.Deserialize(in);
		}

	};
}