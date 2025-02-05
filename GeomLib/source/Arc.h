#pragma once
#include "Circle.h"

namespace geomlib
{
	FLOATING(T)
		class Arc : Circle<T>
	{
	protected:
		Point<T> m_ptStart;
		Point<T> m_ptEnd;
	public:
		Arc() : Circle() {};
		Arc(const Point<T>& center, const Vector<T>& axis, T radius, const Point<T>& start, const Point<T>& end)
		{
			this->m_ptCenter = center;
			this->m_vecAxis = axis;
			this->m_dblRadius = radius;
			m_ptStart = start;
			m_ptEnd = end;
		}
		inline Point<T> Start() const { return m_ptStart; }
		inline void SetStart(const Point<T>& start) { m_ptStart = start; }
		inline Point<T> End() const { return m_ptEnd; }
		inline void SetEnd(const Point<T>& end) { m_ptEnd = end; }

		Circle<T> AsCircle() const
		{
			return Circle<T>(this->Center(), this->Axis(), this->Radius());
		}

		bool Belongs(const Point<T>& pt, T epsPow2 = Epsilon::EpsPow2()) const
		{
			if (!AsCircle().Belongs(pt, epsPow2)) return false;
			Vector<T> start = m_ptStart - this->m_ptCenter;
			Vector<T> end = m_ptEnd - this->m_ptCenter;
			Vector<T> toPoint = pt - this->m_ptCenter;
			return start.FullAngle(end, this->Axis()) >= start.FullAngle(toPoint, this->Axis()) - epsPow2;
		}

		bool IsInSegment(const Point<T>& pt) const
		{
			Vector<T> start = m_ptStart - this->m_ptCenter;
			Vector<T> end = m_ptEnd - this->m_ptCenter;
			Vector<T> toPoint = pt - m_ptCenter;
			return  m_vecAxis.IsOrthogonal(toPoint) &&
					toPoint.Length2() <= m_dblRadius * m_dblRadius + Epsilon::Eps() &&
					start.FullAngle(end, this->Axis()) >= start.FullAngle(toPoint, this->Axis()) - Epsilon::Eps();
		}

		//Returns false if pt is not on circle, else assigns parameter of pt to param
		bool GetParameter(const Point<T>& pt, T& param, T eps = Epsilon::Eps()) const
		{
			if (!Belongs(pt, eps * eps))
				return false;
			Vector<T> start = m_ptStart - this->m_ptCenter;
			Vector<T> toPoint = pt - m_ptCenter;
			param = start.FullAngle(toPoint, m_vecAxis);
			return true;
		}

		Point<T> GetPointByParameter(T param) const
		{
			Vector<T> start = m_ptStart - this->m_ptCenter;
			return start.Rotate(m_vecAxis, param);
		}

		//Returns false if pt is not on arc, else assigns tangent vector to tang
		bool GetTangentIn(const Point<T>& pt, Vector<T>& tang) const
		{
			if (!Belongs(pt, eps * eps))
				return false;
			tang = m_vecAxis.CrossProduct(pt - m_ptCenter).Normalize();
			return true;
		}

		std::string ToString() const
		{
			std::stringstream out;
			out << "Arc with center: ";
			out << this->m_ptCenter.ToString();
			out << "    Axis: ";
			out << this->m_vecAxis.ToString();
			out << "    Radius: ";
			out << this->m_dblRadius.ToString();
			out << "    Start point: ";
			out << m_ptStart.ToString();
			out << "    End point: ";
			out << m_ptEnd.ToString();
			return out.str();
		}
		void Serialize(std::ostream& out) const
		{
			this->m_ptCenter.Serialize(out);
			this->m_vecAxis.Serialize(out);
			out.write((char*)&m_dblRadius, sizeof(T));
			m_ptStart.Serialize(out);
			m_ptEnd.Serialize(out);
		}

		void Deserialize(std::istream& in)
		{
			this->m_ptCenter.Deserialize(in);
			this->m_vecAxis.Deserialize(in);
			in.read((char*)&m_dblRadius, sizeof(T));
			m_ptStart.Deserialize(in);
			m_ptEnd.Deserialize(in);
		}
	};
}