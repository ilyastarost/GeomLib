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
			Vector<T> norm = tmp - lin.Start();
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

		//Projects on the nearest point, returns pt if pt is on axis
		Point<T> ProjectionOf(const Point<T>& pt) const
		{
			if (Line<T>(this->Start(), Direction()).Belongs(pt)) return pt;
			//projection of pt on axis
			Point<T> proj = Line<T>(this->Start(), Direction()).FindNearestPointToLine(pt);
			Vector<T> tmp = pt - proj;
			tmp.Normalize() *= this->Radius();
			return proj + tmp;
		}

		DERIVED_FROM_LINE(S)
		bool IsTangent(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			//ort is orthogonal to lin & passes through start point of cylinder
			Plane<T> ort(this->Start, lin.Direction());
			//finding distance between the cylinder axis and the point of intersection of ort and lin
			return (abs(Line<T>(this->Start(), Direction()).DistanceToLine(ort.FindIntersections(lin)) - Radius()) <= eps);
		}

		DERIVED_FROM_LINE(S)
		std::vector<Point<T>> FindIntersections(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			//if cylinder axis and lin are parallel, returns 0 points
			if (Direction().IsParallel(lin.Direction(), eps * eps)) return std::vector<Point<T>>();
			//if lin is tangent to cylinder, returns 1 point
			if (IsTangent(lin)) return { FindTangentIntersection(lin, eps) };
			//elsewise lin either don't cross cylinder or crosses in 2 points
			auto ans = FindPointsOfIntersection(*this, Line<T>(lin.Start(), lin.Direction()));
			std::vector<Point<T>> res;
			for (auto p : ans) if (lin.Belongs(p)) res.push_back(p);
			return res;
		}

		DERIVED_FROM_LINE(S)
		std::vector<Point<T>> FindTangentIntersection(const S<T>& lin, T eps = Epsilon::Eps()) const
		{
			if (!IsTangent(lin, eps)) return std::vector<Point<T>>();
			//ort is orthogonal to lin & passes through start point of cylinder
			Plane<T> ort(this->Start, lin.Direction());	
			return ort.FindIntersections(lin);
		}


		bool Belongs(const Point<T>& pt, T eps = Epsilon::Eps()) const
		{
			return (abs(Line<T>(this->Start(), Direction()).DistanceToLine(pt) - Radius()) <= eps);
		}

		bool GetParameters(const Point<T>& pt, T& param1, T& param2, T eps = Epsilon::Eps()) const {
			if (!Belongs(pt)) return false;
			//vector with angle parameter = 0
			Vector<T> zeroAngle = Direction().GetOrthogonal() * Radius();
			//ort is orthogonal to cylinder axis & passes through pt
			Plane<T> ort(pt, Direction());
			Point<T> tmp = ort.ProjectionOf(this->Start());
			param1 = tmp.Distance(this->Start()) / Direction().Length();
			//angle with respect to direction of axis
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
			Point<T> proj = Line<T>(this->Start(), Direction()).FindNearestPointToLine(pt);
			norm = pt - proj;
			return true;
		}

		std::string ToString() const
		{
			std::stringstream out;
			out << "Cylinder with center: ";
			out << this->m_ptStart.ToString();
			out << "    Axis: ";
			out << m_vecDirection.ToString();
			out << "    Radius: ";
			out << m_dblRadius << std::endl;
			return out.str();
		}
		void Serialize(std::ostream& out) const
		{
			this->m_ptStart.Serialize(out);
			m_vecDirection.Serialize(out);
			out.write((char*)&m_dblRadius, sizeof(T));
		}

		void Deserialize(std::istream& in)
		{
			this->m_ptStart.Deserialize(in);
			m_vecDirection.Deserialize(in);
			in.read((char*)&m_dblRadius, sizeof(T));
		}

	};
}