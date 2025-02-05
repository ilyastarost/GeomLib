#pragma once
#include "Generic.h"

namespace geomlib
{
	FLOATING(T)
	class Circle
	{
	protected:
		Point<T> m_ptCenter;
		Vector<T> m_vecAxis;
		T m_dblRadius;
	public:
		Circle() = default;
		Circle(const Point<T>& center, const Vector<T>& axis, T radius) : m_ptCenter(center), m_vecAxis(axis), m_dblRadius(radius) {};
		inline Point<T> Center() const { return m_ptCenter; }
		inline void SetCenter(const Point<T>& center) { m_ptCenter = center; }
		inline Vector<T> Axis() const { return m_vecAxis; }
		inline void SetAxis(const Vector<T>& axis) { m_vecAxis = axis; }
		inline T Radius() const { return m_dblRadius; }
		inline void SetRadius(T radius) { m_dblRadius = radius; }


		bool Belongs(const Point<T>& pt, T epsPow2 = Epsilon::EpsPow2()) const
		{
			Vector<T> toPoint = pt - m_ptCenter;
			return m_vecAxis.IsOrthogonal(toPoint, epsPow2) && Epsilon::IsZero(toPoint.Length2() - m_dblRadius * m_dblRadius));
		}

		bool IsInCircle(const Point<T>& pt, T eps = Epsilon::Eps()) const
		{
			Vector<T> toPoint = pt - m_ptCenter;
			return m_vecAxis.IsOrthogonal(toPoint) && toPoint.Length2() <= m_dblRadius * m_dblRadius + eps;
		}

		//Returns false if pt is not on circle, else assigns parameter of pt to param
		bool GetParameter(const Point<T>& pt, T& param, T eps = Epsilon::Eps()) const
		{
			if (!Belongs(pt, eps * eps)) 
				return false;
			Vector<T> toPoint = pt - m_ptCenter;
			param = m_vecAxis.GetOrthogonal().FullAngle(toPoint, m_vecAxis);
			return true;
		}

		Point<T> GetPointByParameter(T param) const
		{
			return m_ptCenter + m_vecAxis.GetOrthogonal().Rotate(m_vecAxis, param) * m_dblRadius;
		}

		//Returns false if pt is not on circle, else assigns tangent vector to tang
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
			out << "Circle with center: ";
			out << m_ptCenter.ToString();
			out << "    Axis: ";
			out << m_vecAxis.ToString();
			out << "    Radius: ";
			out << m_dblRadius;
			return out.str();
		}
		void Serialize(std::ostream& out) const
		{
			m_ptCenter.Serialize(out);
			m_vecAxis.Serialize(out);
			out.write(&m_dblRadius, sizeof(T));
		}

		void Deserialize(std::istream& in)
		{
			m_ptCenter.Deserialize(in);
			m_vecAxis.Deserialize(in);
			in.read((char*)&m_dblRadius, sizeof(T));
		}

	};
}