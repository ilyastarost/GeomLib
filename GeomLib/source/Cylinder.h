#pragma once
#include "Surface.h"
#include "Plane.h"
#include <utility>

namespace geomlib
{
	FLOATING(T)
	class Cylinder : public Surface<T> {
	protected:
		Vector<T> m_vecDirection;
		T m_dblRadius;
		static std::vector<Point<T>> FindPointsOfIntersection(const Cylinder<T>& cyl, const Line<T>& lin) {
			Point<T> tmp = cyl.ProjectionOf(lin.Start());

			//normal vector from surface to point
			Vector<T> norm(tmp.X() - lin.Start().X(), tmp.Y() - lin.Start().Y(), tmp.Z() - lin.Start().Z());

			//check if start point of line is inside of cylinder
			bool inside = (Line<T>(cyl.Start(), cyl.Direction()).DistanceToLine(lin.Start()) < cyl.Radius());

			//angle between normal vector and direction of line
			T angle = std::min(norm.Angle(lin.Direction()), acos(-1) - norm.Angle(lin.Direction()));

			//discriminant for cos formula
			T D = cyl.Radius() * cyl.Radius() - ((inside ? -1 : 1) * norm.Length() + cyl.Radius()) * ((inside ? -1 : 1) * norm.Length() + cyl.Radius()) * sin(angle) * sin(angle);

			if (D < 0) return std::vector<Point<T>>();
			D = sqrt(D);
			T len1 = ((inside ? -1 : 1) * norm.Length() + cyl.Radius()) * cos(angle);
			T len2 = len1 + D;
			len1 -= D;
			Point<T> ans1 = lin.Start() + lin.Direction().NormalizedCopy() * len1;
			Point<T> ans2 = lin.Start() + lin.Direction().NormalizedCopy() * len2;
			return { ans1, ans2 };
		}
	public:
		Cylinder() : Surface<T>() {};
		Cylinder(const Point<T>& pt, const Vector<T>& dir, T radius)
		{
			this->m_ptStart = pt;
			m_vecDirection = dir.NormalizedCopy();
			m_dblRadius = radius;
		}

		inline Vector<T> Direction() const { return m_vecDirection; }
		inline T Radius() const { return m_dblRadius; }
		inline void SetDirection(const Vector<T>& dir) { m_vecDirection = dir; m_vecDirection.Normalize(); }
		inline void SetRadius(T radius) { m_dblRadius = radius; }

		Point<T> ProjectionOf(const Point<T>& pt) const
		{
			Point<T> proj = Line<T>(this->Start(), this->Direction()).FindNearestPointToLine(pt);
			Vector<T> tmp(pt.X() - proj.X(), pt.Y() - proj.Y(), pt.Z() - proj.Z());
			tmp.Normalize() *= this->Radius();
			return proj + tmp;
		}

		DERIVED_FROM_LINE(S)
		bool IsTangent(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			Plane<T> ort(this->Start, lin.Direction());
			return (Line<T>(this->Start(), this->Direction()).DistanceToLine(ort.FindIntersection(lin)) <= eps);
		}

		DERIVED_FROM_LINE(S)
		std::vector<Point<T>> FindIntersections(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			if (this->Direction().IsParallel(lin.Direction(), eps * eps)) return std::vector<Point<T>>();
			if (IsTangent(lin)) return { FindTangentIntersection(lin, eps) };
			auto ans = FindPointsOfIntersection(*this, Line<T>(lin.Start(), lin.Direction()));
			std::vector<Point<T>> res;
			for (auto p : ans) if (lin.Belongs(p)) res.push_back(p);
			return res;
		}

		DERIVED_FROM_LINE(S)
		std::vector<Point<T>> FindTangentIntersection(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			if (!IsTangent(lin, eps)) return std::vector<Point<T>>();
			Plane<T> ort(this->Start, lin.Direction());
			return ort.FindIntersections(lin);
		}


		bool Belongs(const Point<T>& pt, T eps = Epsilon::Eps()) const
		{
			return (abs(Line<T>(this->Start(), this->Direction()).DistanceToLine(pt) - this->Radius()) <= eps);
		}

		bool GetParameters(const Point<T>& pt, T& param1, T& param2, T eps = Epsilon::Eps()) const {
			if (!Belongs(pt)) return false;
			Vector<T> zeroAngle = Direction().GetOrthogonal() * Radius();
			Plane<T> ort(pt, Direction());
			Point<T> tmp = ort.ProjectionOf(this->Start());
			param1 = tmp.Distance(this->Start()) / Direction().Length();
			param2 = zeroAngle.FullAngle(pt - tmp, Direction());
			return true;
		}

		Point<T> GetPointByParameters(T param1, T param2) const
		{
			Vector<T> vec = Direction().GetOrthogonal() * Radius();
			return this->Start() + vec.Rotate(Direction(), param2) + Direction() * param1;
		}

		bool GetNormalIn(const Point<T>& pt, Vector<T>& norm) const
		{
			if (!Belongs(pt)) return false;
			Point<T> proj = Line<T>(this->Start(), this->Direction()).FindNearestPointToLine(pt);
			norm = pt - proj;
			return true;
		}

	};
}