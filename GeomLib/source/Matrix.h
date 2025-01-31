#pragma once
#include "Generic.h"
#include <vector>
#include <cmath>

namespace geomlib
{
	FLOATING(T)
	class Matrix
	{
	protected:
		std::vector<std::vector<T>> m_vec2Matr;
        bool OptimizedInvertion(Matrix<T>& res) const
        {
            std::vector<std::vector<T>> b = m_vec2Matr;
            std::swap(b[0][1], b[1][0]);
            std::swap(b[0][2], b[2][0]);
            std::swap(b[1][2], b[2][1]);
            for (int i = 0; i < 3; i++) {
                b[3][i] = 0;
                for (int j = 0; j < 3; j++) {
                    b[3][i] -= m_vec2Matr[i][j] * m_vec2Matr[3][j];
                }
            }
            res.m_vec2Matr = b;
            return true;
        }
	public:
		Matrix()
		{
			m_vec2Matr = std::vector<std::vector<T>>(4, std::vector<T>(4, 0));
			for (int i = 0; i < 4; i++)
				m_vec2Matr[i][i] = 1;
		}

        std::vector<std::vector<T>> Matr() const
        {
            return m_vec2Matr;
        }

		bool Invertion(Matrix<T>& res) const
		{
            if (abs(res.Matr()[0][3]) <= Epsilon::Eps() && abs(res.Matr()[1][3]) <= Epsilon::Eps() && abs(res.Matr()[2][3]) <= Epsilon::Eps() && abs(res.Matr()[3][3] - 1) <= Epsilon::Eps())
                return OptimizedInvertion(res);
            T s0 = m_vec2Matr[0][0] * m_vec2Matr[1][1] - m_vec2Matr[1][0] * m_vec2Matr[0][1];
            T s1 = m_vec2Matr[0][0] * m_vec2Matr[1][2] - m_vec2Matr[1][0] * m_vec2Matr[0][2];
            T s2 = m_vec2Matr[0][0] * m_vec2Matr[1][3] - m_vec2Matr[1][0] * m_vec2Matr[0][3];
            T s3 = m_vec2Matr[0][1] * m_vec2Matr[1][2] - m_vec2Matr[1][1] * m_vec2Matr[0][2];
            T s4 = m_vec2Matr[0][1] * m_vec2Matr[1][3] - m_vec2Matr[1][1] * m_vec2Matr[0][3];
            T s5 = m_vec2Matr[0][2] * m_vec2Matr[1][3] - m_vec2Matr[1][2] * m_vec2Matr[0][3];

            T c5 = m_vec2Matr[2][2] * m_vec2Matr[3][3] - m_vec2Matr[3][2] * m_vec2Matr[2][3];
            T c4 = m_vec2Matr[2][1] * m_vec2Matr[3][3] - m_vec2Matr[3][1] * m_vec2Matr[2][3];
            T c3 = m_vec2Matr[2][1] * m_vec2Matr[3][2] - m_vec2Matr[3][1] * m_vec2Matr[2][2];
            T c2 = m_vec2Matr[2][0] * m_vec2Matr[3][3] - m_vec2Matr[3][0] * m_vec2Matr[2][3];
            T c1 = m_vec2Matr[2][0] * m_vec2Matr[3][2] - m_vec2Matr[3][0] * m_vec2Matr[2][2];
            T c0 = m_vec2Matr[2][0] * m_vec2Matr[3][1] - m_vec2Matr[3][0] * m_vec2Matr[2][1];

            T D = (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);
            if (abs(D) <= Epsilon::Eps()) return false;
            T invdet = 1.0 / D;

            std::vector<std::vector<T>> b(4, std::vector<T>(4));

            b[0][0] = (m_vec2Matr[1][1] * c5 - m_vec2Matr[1][2] * c4 + m_vec2Matr[1][3] * c3) * invdet;
            b[0][1] = (-m_vec2Matr[0][1] * c5 + m_vec2Matr[0][2] * c4 - m_vec2Matr[0][3] * c3) * invdet;
            b[0][2] = (m_vec2Matr[3][1] * s5 - m_vec2Matr[3][2] * s4 + m_vec2Matr[3][3] * s3) * invdet;
            b[0][3] = (-m_vec2Matr[2][1] * s5 + m_vec2Matr[2][2] * s4 - m_vec2Matr[2][3] * s3) * invdet;

            b[1][0] = (-m_vec2Matr[1][0] * c5 + m_vec2Matr[1][2] * c2 - m_vec2Matr[1][3] * c1) * invdet;
            b[1][1] = (m_vec2Matr[0][0] * c5 - m_vec2Matr[0][2] * c2 + m_vec2Matr[0][3] * c1) * invdet;
            b[1][2] = (-m_vec2Matr[3][0] * s5 + m_vec2Matr[3][2] * s2 - m_vec2Matr[3][3] * s1) * invdet;
            b[1][3] = (m_vec2Matr[2][0] * s5 - m_vec2Matr[2][2] * s2 + m_vec2Matr[2][3] * s1) * invdet;

            b[2][0] = (m_vec2Matr[1][0] * c4 - m_vec2Matr[1][1] * c2 + m_vec2Matr[1][3] * c0) * invdet;
            b[2][1] = (-m_vec2Matr[0][0] * c4 + m_vec2Matr[0][1] * c2 - m_vec2Matr[0][3] * c0) * invdet;
            b[2][2] = (m_vec2Matr[3][0] * s4 - m_vec2Matr[3][1] * s2 + m_vec2Matr[3][3] * s0) * invdet;
            b[2][3] = (-m_vec2Matr[2][0] * s4 + m_vec2Matr[2][1] * s2 - m_vec2Matr[2][3] * s0) * invdet;

            b[3][0] = (-m_vec2Matr[1][0] * c3 + m_vec2Matr[1][1] * c1 - m_vec2Matr[1][2] * c0) * invdet;
            b[3][1] = (m_vec2Matr[0][0] * c3 - m_vec2Matr[0][1] * c1 + m_vec2Matr[0][2] * c0) * invdet;
            b[3][2] = (-m_vec2Matr[3][0] * s3 + m_vec2Matr[3][1] * s1 - m_vec2Matr[3][2] * s0) * invdet;
            b[3][3] = (m_vec2Matr[2][0] * s3 - m_vec2Matr[2][1] * s1 + m_vec2Matr[2][2] * s0) * invdet;

            res.m_vec2Matr = b;
            return true;
		}

        static Matrix<T> TranslationInit(const Vector<T>& move)
        {
            Matrix<T> res;
            res.m_vec2Matr[3][0] = move.X();
            res.m_vec2Matr[3][1] = move.Y();
            res.m_vec2Matr[3][2] = move.Z();
            return res;
        }

        static Matrix<T> ScalingInit(const Vector<T>& scale)
        {
            Matrix<T> res;
            res.m_vec2Matr[0][0] = scale.X();
            res.m_vec2Matr[1][1] = scale.Y();
            res.m_vec2Matr[2][2] = scale.Z();
            return res;
        }

        static Matrix<T> RotationAroundXInit(T angle)
        {
            Matrix<T> res;
            res.m_vec2Matr[1][1] = cos(angle);
            res.m_vec2Matr[2][2] = cos(angle);
            res.m_vec2Matr[2][1] = sin(angle);
            res.m_vec2Matr[1][2] = -sin(angle);
            return res;
        }

        static Matrix<T> RotationAroundYInit(T angle)
        {
            Matrix<T> res;
            res.m_vec2Matr[0][0] = cos(angle);
            res.m_vec2Matr[2][2] = cos(angle);
            res.m_vec2Matr[2][0] = sin(angle);
            res.m_vec2Matr[0][2] = -sin(angle);
            return res;
        }

        static Matrix<T> RotationAroundZInit(T angle)
        {
            Matrix<T> res;
            res.m_vec2Matr[0][0] = cos(angle);
            res.m_vec2Matr[1][1] = cos(angle);
            res.m_vec2Matr[1][0] = sin(angle);
            res.m_vec2Matr[0][1] = -sin(angle);
            return res;
        }

        static Matrix<T> RotationInit(const Vector<T>& axis, T angle)
        {
            Matrix<T> res;
            T c = cos(angle);
            T s = sin(angle);
            T C = 1 - c;
            T xs = axis.X() * s;
            T ys = axis.Y() * s;
            T zs = axis.Z() * s;
            T xC = axis.X() * C;
            T yC = axis.Y() * C;
            T zC = axis.Z() * C;
            T xyC = axis.X() * yC;
            T yzC = axis.Y() * zC;
            T zxC = axis.Z() * xC;

            res.m_vec2Matr[0][0] = axis.X() * xC + c; res.m_vec2Matr[0][1] = xyC - zs; res.m_vec2Matr[0][2] = zxC + ys;
            res.m_vec2Matr[1][0] = xyC + zs; res.m_vec2Matr[1][1] = axis.Y() * yC + c; res.m_vec2Matr[1][2] = yzC - xs;
            res.m_vec2Matr[2][0] = zxC - ys; res.m_vec2Matr[2][1] = yzC + xs; res.m_vec2Matr[2][2] = axis.Z() * zC + c;
            return res;
        }

        static bool ToCoordinatesInit(const Vector<T>& a, const Vector<T>& b, const Vector<T>& c, Matrix<T>& res)
        {
            res = Matrix<T>();
            res.m_vec2Matr[0][0] = a.X(); res.m_vec2Matr[0][1] = a.Y(); res.m_vec2Matr[0][2] = a.Z();
            res.m_vec2Matr[1][0] = b.X(); res.m_vec2Matr[1][1] = b.Y(); res.m_vec2Matr[1][2] = b.Z();
            res.m_vec2Matr[2][0] = c.X(); res.m_vec2Matr[2][1] = c.Y(); res.m_vec2Matr[2][2] = c.Z();

            return res.Invertion(res);
        }

        Matrix<T> operator* (const Matrix<T>& rhs) const
        {
            Matrix<T> res;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    res.m_vec2Matr[i][j] = 0;
                    for (int k = 0; k < 4; k++) {
                        res.m_vec2Matr[i][j] += m_vec2Matr[i][k] * rhs.m_vec2Matr[k][j];
                    }
                }
            }
            return res;
        }

        Matrix<T>& operator*= (const Matrix<T>& rhs)
        {
            m_vec2Matr = (*this * rhs).m_vec2Matr;
        }

	};

    DERIVED_FROM_COORDINATES(S, T)
    S<T> operator* (const S<T>& lhs, const Matrix<T>& rhs)
    {
        S<T> res;
        res.SetX(lhs.X() * rhs.Matr()[0][0] + lhs.Y() * rhs.Matr()[1][0] + lhs.Z() * rhs.Matr()[2][0] + rhs.Matr()[3][0]);
        res.SetY(lhs.X() * rhs.Matr()[0][1] + lhs.Y() * rhs.Matr()[1][1] + lhs.Z() * rhs.Matr()[2][1] + rhs.Matr()[3][1]);
        res.SetZ(lhs.X() * rhs.Matr()[0][2] + lhs.Y() * rhs.Matr()[1][2] + lhs.Z() * rhs.Matr()[2][2] + rhs.Matr()[3][2]);
        return res;
    }

    DERIVED_FROM_COORDINATES(S, T)
    S<T>& operator*= (S<T>& lhs, const Matrix<T>& rhs)
    {
        lhs = lhs * rhs;
        return lhs;
    }
}