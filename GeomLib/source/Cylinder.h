#pragma once
#include "Surface.h"
#include "Plane.h"
#include <utility>

namespace geomlib
{
	FLOATING(T)
	class Cylinder : public Surface<T> {
	protected:
		static std::pair<Point<T>, Point<T>> FindPointsOfIntersection(const Cylinder<T>& cyl, const Line<T>& lin) {
			Point<T> tmp = cyl.ProjectionOf(lin.Start());
			//normal vector from surface to point
			Vector<T> norm(tmp.X() - lin.Start().X(), tmp.Y() - lin.Start().Y(), tmp.Z() - lin.Start().Z());
			//angle between normal vector and direction of line
			T angle = std::min(norm.Angle(lin.Direction()), acos(-1) - norm.Angle(lin.Direction()));
			//discriminant for cos formula
			T D = cyl.Radius() * cyl.Radius() - (norm.Length() + cyl.Radius()) * (norm.Length() + cyl.Radius()) * sin(angle) * sin(angle);

			if (D < 0) return { Point<T>(NAN, NAN, NAN), Point<T>(NAN, NAN, NAN) };
			D = sqrt(D);

			T len1 = (norm.Length() + cyl.Radius()) * cos(angle);
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
			this->m_vecBase = dir.NormalizedCopy() * radius;
		}

		inline Vector<T> Direction() const { return this->m_vecBase; }
		inline T Radius() const { return this->m_vecBase.Length(); }
		inline void SetDirection(const Vector<T>& dir) { this->m_vecBase = dir.NormalizedCopy() * this->Radius(); }
		inline void SetRadius(T radius) { this->m_vecBase = this->m_vecBase.NormalizedCopy() * radius; }

		Vector<T> GetNormalInPoint(const Point<T>& pt) const {
			if (!Belongs(pt)) return Vector<T>(NAN, NAN, NAN);
			Point<T> proj = Line<T>(this->Start(), this->Direction()).FindNearestPointToLine(pt);
			return Vector<T>(pt.X() - proj.X(), pt.Y() - proj.Y(), pt.Z() - proj.Z());
		}

		Point<T> ProjectionOf(const Point<T>& pt) const
		{
			Point<T> proj = Line<T>(this->Start(), this->Direction()).FindNearestPointToLine(pt);
			Vector<T> tmp(pt.X() - proj.X(), pt.Y() - proj.Y(), pt.Z() - proj.Z());
			tmp.Normalize() *= this->Radius();
			return proj + tmp;
		}

		DERIVED_FROM_LINE(S)
		bool IsTangent(const S<T>& lin) const
		{
			Plane<T> ort(this->Start, lin.Direction());
			return (Line<T>(this->Start(), this->Direction()).DistanceToLine(ort.FindIntersection(lin)) <= Epsilon::Eps());
		}

		DERIVED_FROM_LINE(S)
		bool Intersects(const S<T>& lin) const
		{
			if (this->Direction().IsParallel(lin.Direction())) return false;
			auto ans = FindPointsOfIntersection(*this, Line<T>(lin.Start(), lin.Direction()));
			if (ans.first.X() == NAN) return false;
			return lin.Belongs(ans.first);
		}

		DERIVED_FROM_LINE(S)
		Point<T> FindTangentIntersection(const S<T>& lin) const
		{
			if (!IsTangent(lin)) return Point<T>(NAN, NAN, NAN);
			Plane<T> ort(this->Start, lin.Direction());
			return ort.FindIntersection(lin);
		}

		DERIVED_FROM_LINE(S)
		std::pair<Point<T>, Point<T>> FindIntersections(const S<T>& lin) const
		{
			if (Intersects(lin)) return FindPointsOfIntersection(*this, Line<T>(lin.Start(), lin.Direction()));
			return { Point<T>(NAN, NAN, NAN), Point<T>(NAN, NAN, NAN) };
		}

		bool Belongs(const Point<T>& pt) const
		{
			return (abs(this->Direction().DistanceToLine(pt) - this->Radius()) <= Epsilon::Eps);
		}

		std::pair<T, T> GetParameters(const Point<T>& pt, const Vector<T>& dir, const Vector<T>& zeroAngle) {
			if (!dir.IsParallel(this->Direction())) return { NAN, NAN };
			if (!Belongs(pt)) return { NAN, NAN };
			if (!Belongs(this->Start() + zeroAngle)) return { NAN, NAN };
			Plane<T> ort(pt, dir);
			Point<T> tmp = ort.ProjectionOf(this->Start());
			T t = tmp.Distance(this->Start()) / dir.Length();
			T s = zeroAngle.Angle(Vector<T>(pt.X() - tmp.X(), pt.Y() - tmp.Y(), pt.Z() - tmp.Z()));
			return { t, s };
		}

	};
}