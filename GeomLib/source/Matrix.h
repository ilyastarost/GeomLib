#pragma once
#include "Generic.h"
#include <iomanip>
#include <vector>
#include <cmath>

namespace geomlib
{
	FLOATING(T)
	class Matrix
	{
	protected:
        T m_data[16];
	public:
        Matrix() 
        {
            memset(m_data, 0, 16 * sizeof(T));
        }

        Matrix(T *nums)
        {
            memcpy(m_data, nums, 16 * sizeof(T));
        }

        const T* Matr() const
        {
            return m_data;
        }

        bool operator== (const Matrix<T>& rhs) const
        {
            bool res = true;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    if (abs(m_data[i * 4 + j] - rhs.m_data[i * 4 + j]) > Epsilon::Eps()) {
                        res = false;
                        break;
                    }
                }
            }
            return res;
        }

        static Matrix<T> GetZero()
        {
            return Matrix<T>();
        }

        static Matrix<T> GetIdentity()
        {
            T forId[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
            return Matrix<T>(forId);
        }

		Matrix<T> InvertedCopy() const
		{
            Matrix<T> res = GetZero();
            //upper 2x2 minors
            T s0 = m_data[0 * 4 + 0] * m_data[1 * 4 + 1] - m_data[1 * 4 + 0] * m_data[0 * 4 + 1];
            T s1 = m_data[0 * 4 + 0] * m_data[1 * 4 + 2] - m_data[1 * 4 + 0] * m_data[0 * 4 + 2];
            T s2 = m_data[0 * 4 + 0] * m_data[1 * 4 + 3] - m_data[1 * 4 + 0] * m_data[0 * 4 + 3];
            T s3 = m_data[0 * 4 + 1] * m_data[1 * 4 + 2] - m_data[1 * 4 + 1] * m_data[0 * 4 + 2];
            T s4 = m_data[0 * 4 + 1] * m_data[1 * 4 + 3] - m_data[1 * 4 + 1] * m_data[0 * 4 + 3];
            T s5 = m_data[0 * 4 + 2] * m_data[1 * 4 + 3] - m_data[1 * 4 + 2] * m_data[0 * 4 + 3];
            //lower 2x2 minors
            T c5 = m_data[2 * 4 + 2] * m_data[3 * 4 + 3] - m_data[3 * 4 + 2] * m_data[2 * 4 + 3];
            T c4 = m_data[2 * 4 + 1] * m_data[3 * 4 + 3] - m_data[3 * 4 + 1] * m_data[2 * 4 + 3];
            T c3 = m_data[2 * 4 + 1] * m_data[3 * 4 + 2] - m_data[3 * 4 + 1] * m_data[2 * 4 + 2];
            T c2 = m_data[2 * 4 + 0] * m_data[3 * 4 + 3] - m_data[3 * 4 + 0] * m_data[2 * 4 + 3];
            T c1 = m_data[2 * 4 + 0] * m_data[3 * 4 + 2] - m_data[3 * 4 + 0] * m_data[2 * 4 + 2];
            T c0 = m_data[2 * 4 + 0] * m_data[3 * 4 + 1] - m_data[3 * 4 + 0] * m_data[2 * 4 + 1];

            T D = (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);
            if (abs(D) <= Epsilon::Eps()) 
                return Matrix<T>();
            T invdet = 1.0 / D;

            res.m_data[0 * 4 + 0] =  (m_data[1 * 4 + 1] * c5 - m_data[1 * 4 + 2] * c4 + m_data[1 * 4 + 3] * c3) * invdet;
            res.m_data[0 * 4 + 1] = (-m_data[0 * 4 + 1] * c5 + m_data[0 * 4 + 2] * c4 - m_data[0 * 4 + 3] * c3) * invdet;
            res.m_data[0 * 4 + 2] =  (m_data[3 * 4 + 1] * s5 - m_data[3 * 4 + 2] * s4 + m_data[3 * 4 + 3] * s3) * invdet;
            res.m_data[0 * 4 + 3] = (-m_data[2 * 4 + 1] * s5 + m_data[2 * 4 + 2] * s4 - m_data[2 * 4 + 3] * s3) * invdet;

            res.m_data[1 * 4 + 0] = (-m_data[1 * 4 + 0] * c5 + m_data[1 * 4 + 2] * c2 - m_data[1 * 4 + 3] * c1) * invdet;
            res.m_data[1 * 4 + 1] =  (m_data[0 * 4 + 0] * c5 - m_data[0 * 4 + 2] * c2 + m_data[0 * 4 + 3] * c1) * invdet;
            res.m_data[1 * 4 + 2] = (-m_data[3 * 4 + 0] * s5 + m_data[3 * 4 + 2] * s2 - m_data[3 * 4 + 3] * s1) * invdet;
            res.m_data[1 * 4 + 3] =  (m_data[2 * 4 + 0] * s5 - m_data[2 * 4 + 2] * s2 + m_data[2 * 4 + 3] * s1) * invdet;

            res.m_data[2 * 4 + 0] =  (m_data[1 * 4 + 0] * c4 - m_data[1 * 4 + 1] * c2 + m_data[1 * 4 + 3] * c0) * invdet;
            res.m_data[2 * 4 + 1] = (-m_data[0 * 4 + 0] * c4 + m_data[0 * 4 + 1] * c2 - m_data[0 * 4 + 3] * c0) * invdet;
            res.m_data[2 * 4 + 2] =  (m_data[3 * 4 + 0] * s4 - m_data[3 * 4 + 1] * s2 + m_data[3 * 4 + 3] * s0) * invdet;
            res.m_data[2 * 4 + 3] = (-m_data[2 * 4 + 0] * s4 + m_data[2 * 4 + 1] * s2 - m_data[2 * 4 + 3] * s0) * invdet;

            res.m_data[3 * 4 + 0] = (-m_data[1 * 4 + 0] * c3 + m_data[1 * 4 + 1] * c1 - m_data[1 * 4 + 2] * c0) * invdet;
            res.m_data[3 * 4 + 1] =  (m_data[0 * 4 + 0] * c3 - m_data[0 * 4 + 1] * c1 + m_data[0 * 4 + 2] * c0) * invdet;
            res.m_data[3 * 4 + 2] = (-m_data[3 * 4 + 0] * s3 + m_data[3 * 4 + 1] * s1 - m_data[3 * 4 + 2] * s0) * invdet;
            res.m_data[3 * 4 + 3] =  (m_data[2 * 4 + 0] * s3 - m_data[2 * 4 + 1] * s1 + m_data[2 * 4 + 2] * s0) * invdet;

            return res;
		}

        void Invert()
        {
            memcpy(m_data, InvertedCopy().m_data, 16 * sizeof(T));
        }

        static Matrix<T> TranslationInit(const Vector<T>& move)
        {
            Matrix<T> res = GetIdentity();
            res.m_data[3 * 4 + 0] = move.X();
            res.m_data[3 * 4 + 1] = move.Y();
            res.m_data[3 * 4 + 2] = move.Z();
            return res;
        }

        static Matrix<T> ScalingInit(const Vector<T>& scale)
        {
            Matrix<T> res = GetIdentity();

            res.m_data[3 * 4 + 3] = 1;

            res.m_data[0 * 4 + 0] = scale.X();
            res.m_data[1 * 4 + 1] = scale.Y();
            res.m_data[2 * 4 + 2] = scale.Z();
            return res;
        }

        static Matrix<T> RotationAroundXInit(T angle)
        {
            Matrix<T> res = GetIdentity();
            res.m_data[1 * 4 + 1] = cos(angle);
            res.m_data[2 * 4 + 2] = cos(angle);
            res.m_data[2 * 4 + 1] = -sin(angle);
            res.m_data[1 * 4 + 2] = sin(angle);
            return res;
        }

        static Matrix<T> RotationAroundYInit(T angle)
        {
            Matrix<T> res = GetIdentity();
            res.m_data[0 * 4 + 0] = cos(angle);
            res.m_data[2 * 4 + 2] = cos(angle);
            res.m_data[2 * 4 + 0] = sin(angle);
            res.m_data[0 * 4 + 2] = -sin(angle);
            return res;
        }

        static Matrix<T> RotationAroundZInit(T angle)
        {
            Matrix<T> res = GetIdentity();
            res.m_data[0 * 4 + 0] = cos(angle);
            res.m_data[1 * 4 + 1] = cos(angle);
            res.m_data[1 * 4 + 0] = -sin(angle);
            res.m_data[0 * 4 + 1] = sin(angle);
            return res;
        }

        static Matrix<T> RotationInit(const Vector<T>& axis, T angle)
        {
            Vector<T> tmp = axis.NormalizedCopy();
            Matrix<T> res = GetIdentity();
            T c = cos(angle);
            T s = sin(angle);
            T C = 1 - c;
            T xs = tmp.X() * s;
            T ys = tmp.Y() * s;
            T zs = tmp.Z() * s;
            T xC = tmp.X() * C;
            T yC = tmp.Y() * C;
            T zC = tmp.Z() * C;
            T xyC = tmp.X() * yC;
            T yzC = tmp.Y() * zC;
            T zxC = tmp.Z() * xC;

            res.m_data[0 * 4 + 0] = tmp.X() * xC + c; res.m_data[0 * 4 + 1] = xyC + zs; res.m_data[0 * 4 + 2] = zxC - ys;
            res.m_data[1 * 4 + 0] = xyC - zs; res.m_data[1 * 4 + 1] = tmp.Y() * yC + c; res.m_data[1 * 4 + 2] = yzC + xs;
            res.m_data[2 * 4 + 0] = zxC + ys; res.m_data[2 * 4 + 1] = yzC - xs; res.m_data[2 * 4 + 2] = tmp.Z() * zC + c;
            return res;
        }

        static Matrix<T> ToCoordinatesInit(const Vector<T>& a, const Vector<T>& b, const Vector<T>& c)
        {
            Matrix<T> res = GetIdentity();
            res.m_data[0 * 4 + 0] = a.X(); res.m_data[0 * 4 + 1] = a.Y(); res.m_data[0 * 4 + 2] = a.Z();
            res.m_data[1 * 4 + 0] = b.X(); res.m_data[1 * 4 + 1] = b.Y(); res.m_data[1 * 4 + 2] = b.Z();
            res.m_data[2 * 4 + 0] = c.X(); res.m_data[2 * 4 + 1] = c.Y(); res.m_data[2 * 4 + 2] = c.Z();

            return res.InvertedCopy();
        }

        Matrix<T> operator* (const Matrix<T>& rhs) const
        {
            Matrix<T> res = GetZero();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    res.m_data[i * 4 + j] = 0;
                    for (int k = 0; k < 4; k++) {
                        res.m_data[i * 4 + j] += m_data[i * 4 + k] * rhs.m_data[k * 4 + j];
                    }
                }
            }
            return res;
        }

        Matrix<T>& operator*= (const Matrix<T>& rhs)
        {
            m_data = (*this * rhs).m_data;
        }

        std::string ToString() const
        {
            std::stringstream out;
            out << "Matrix (" << typeid(T).name() << "):" << std::endl;

            for (int i = 0; i < 4; i++) {
                out << "| ";
                for (int j = 0; j < 4; j++) {
                    out << std::setw(12) << m_data[i * 4 + j] << ' ';
                }
                out << "|" << std::endl;
            }
            return out.str();
        }
        void Serialize(std::ostream& out) const
        {
            out.write((char*)&m_data, 16 * sizeof(T));
        }

        void Deserialize(std::istream& in)
        {
            in.read((char*)&m_data, 16 * sizeof(T));
        }

	};

    DERIVED_FROM_COORDINATES(S, T)
    S<T> operator* (const S<T>& lhs, const Matrix<T>& rhs)
    {
        S<T> res;
        res.SetX(lhs.X() * rhs.Matr()[0 * 4 + 0] + lhs.Y() * rhs.Matr()[1 * 4 + 0] + lhs.Z() * rhs.Matr()[2 * 4 + 0] + rhs.Matr()[3 * 4 + 0] * (typeid(lhs).name() == typeid(Point<T>(0, 0, 0)).name() ? 1 : 0));
        res.SetY(lhs.X() * rhs.Matr()[0 * 4 + 1] + lhs.Y() * rhs.Matr()[1 * 4 + 1] + lhs.Z() * rhs.Matr()[2 * 4 + 1] + rhs.Matr()[3 * 4 + 1] * (typeid(lhs).name() == typeid(Point<T>(0, 0, 0)).name() ? 1 : 0));
        res.SetZ(lhs.X() * rhs.Matr()[0 * 4 + 2] + lhs.Y() * rhs.Matr()[1 * 4 + 2] + lhs.Z() * rhs.Matr()[2 * 4 + 2] + rhs.Matr()[3 * 4 + 2] * (typeid(lhs).name() == typeid(Point<T>(0, 0, 0)).name() ? 1 : 0));
        return res;
    }

    DERIVED_FROM_COORDINATES(S, T)
    S<T>& operator*= (S<T>& lhs, const Matrix<T>& rhs)
    {
        lhs = lhs * rhs;
        return lhs;
    }
}