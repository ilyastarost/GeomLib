#pragma once
#include "Cylinder.h"
#include "Segment.h"
#include "Matrix.h"
#include "Plane.h"
#include "Ray.h"
#include <vector>
#include <set>

namespace geomlib
{
	struct Triangle
	{
		int ind[3];

	};

	FLOATING(T)
	class TessModel
	{
	protected:
		std::vector<Point<T>> m_vecAllPoints;
		std::vector<Vector<T>> m_vecAllNormals;
		std::vector<Triangle> m_vecTriangles;
		std::vector<int> m_vecLastOfSurface;

		void MergeHelper(const std::vector<Point<T>>& pts, const std::vector<Vector<T>>& norms, const std::vector<Triangle>& tr)
		{
			for (int i = 0; i < tr.size(); i++) {
				for (int j = 0; j < 3; j++) {
					tr[i].ind[j] += pts.size();
				}
			}
			m_vecTriangles.insert(m_vecTriangles.end(), tr.begin(), tr.end());
			m_vecAllPoints.insert(m_vecAllPoints.end(), pts.begin(), pts.end());
			m_vecAllNormals.insert(m_vecAllNormals.end(), norms.begin(), norms.end());
		}

		static Vector<T> NormalToCoords(const Point<T>& a, const Point<T>& b, const Point<T>& c)
		{
			return (b - a).CrossProduct(c - a).Normalize();
		}

	public:
		Vector<T> NormalToTriangle(int ind) const
		{
			return NormalToCoords(m_vecAllPoints[m_vecTriangles[ind].ind[0]], m_vecAllPoints[m_vecTriangles[ind].ind[1]], m_vecAllPoints[m_vecTriangles[ind].ind[2]]);
		}

		void MergeModels(const TessModel<T>& model) 
		{
			MergeHelper(model.m_vecAllPoints, model.m_vecAllNormals, model.m_vecTriangles);
			int old = m_vecLastOfSurface.back();
			for (int i = 0; i < model.m_vecLastOfSurface.size(); i++)
			{
				m_vecLastOfSurface.push_back(old + model.m_vecLastOfSurface[i]);
			}
		}

		void AddSurface(const std::vector<Point<T>>& pts, const std::vector<Vector<T>>& norms, const std::vector<Triangle>& tr)
		{
			MergeHelper(pts, norms, tr);
			m_vecLastOfSurface.push_back(m_vecTriangles.size() - 1);
		}

		int GetSurfaceByTriangle(int ind) const
		{
			return std::lower_bound(m_vecLastOfSurface.begin(), m_vecLastOfSurface.end(), ind) - m_vecLastOfSurface.begin();
		}

		std::vector<Point<T>> GetPointsOfTriangle(int ind) {
			std::vector<Point<T>> res = { m_vecAllPoints[m_vecTriangles[ind].ind[0]],
										  m_vecAllPoints[m_vecTriangles[ind].ind[1]],
										  m_vecAllPoints[m_vecTriangles[ind].ind[2]] };
			return res;
		}

		static bool IntersectsTriangle(int ind, const Ray<T>& ray, Point<T>& pt)
		{
			auto check = Plane<T>(m_vecTriangles[ind].ind[0], NormalToTriangle(ind)).FindIntersections(ray);
			if (check.size() == 0) 
				return false;
			Point<T> p = check[0];
			Vector<T> n1 = NormalToCoords(m_vecTriangles[ind].ind[0], m_vecTriangles[ind].ind[1], p);
			Vector<T> n2 = NormalToCoords(m_vecTriangles[ind].ind[1], m_vecTriangles[ind].ind[2], p);
			Vector<T> n3 = NormalToCoords(m_vecTriangles[ind].ind[2], m_vecTriangles[ind].ind[0], p);

			if (!Epsilon::IsZero(n1.DotProduct(n2) - 1) || !Epsilon::IsZero(n1.DotProduct(n3) - 1)) return false;
			pt = p;
			return true;
		}

		bool FindIntersection(const Ray<T>& ray, Point<T>& pt, int& ind) const
		{
			Point<T> ans;
			int pos = -1;
			bool found = false;
			for (int i = 0; i < m_vecTriangles.size(); i++)
			{
				Point<T> cur;
				if (IntersectsTriangle(i, ray, cur))
				{
					if (!found || cur.Distance2(ray.Start()) < ans.Distance2(ray.Start())) {
						ans = cur;
						pos = i;
						found = true;
					}
				}
			}
			if (!found)
				return false;
			pt = ans;
			ind = pos;
			return true;
		}

		void SplitCylinder(const Cylinder<T>& cyl, T h, T deviation)
		{
			int n = acos(-1) / acos(1 - deviation / cyl.Radius()) + 1;
			T angle = 2 * acos(-1) / n;
			Vector<T> cur = cyl.Direction().GetOrthogonal() * cyl.Radius();
			Vector<T> norm = cyl.Direction().Opposite();
			for (int i = 0; i < n; i++)
			{
				m_vecAllPoints.push_back(cyl.Start() + cur);
				m_vecAllNormals.push_back(norm);
				cur = cur.Rotate(cyl.Direction(), angle);
			}
			m_vecAllPoints.push_back(cyl.Start());
			m_vecAllNormals.push_back(norm);
			Vector<T> toUpper = cyl.Direction().NormalizedCopy() * h;
			norm = cyl.Direction();
			for (int i = 0; i <= n; i++)
			{
				m_vecAllPoints.push_back(m_vecAllPoints[i] + toUpper);
				m_vecAllNormals.push_back(norm);
			}
			for (int i = 0; i < n; i++)
			{
				m_vecTriangles.push_back({ i, (i + 1) % n, n });
			}
			m_vecLastOfSurface.push_back(n - 1);

			for (int i = 0; i < n; i++)
			{
				m_vecTriangles.push_back({ n + 1 + i, n + 1 + (i + 1) % n, 2 * n + 1});
			}
			m_vecLastOfSurface.push_back(2 * n - 1);

			m_vecAllPoints.insert(m_vecAllPoints.end(), m_vecAllPoints.begin(), m_vecAllPoints.end());
			for (int i = 0; i < n; i++)
			{
				m_vecTriangles.push_back({ 2 * (n + 1) + i, 2 * (n + 1) + (i + 1) % n, 3 * (n + 1) + i });
				m_vecTriangles.push_back({ 3 * (n + 1) + (i + 1) % n, 3 * (n + 1) + i, 2 * (n + 1) + (i + 1) % n });
			}
			m_vecLastOfSurface.push_back(4 * n - 1);

			for (int i = 2 * (n + 1); i < 3 * (n + 1); i++)
			{
				m_vecAllNormals.push_back(m_vecAllPoints[i] - m_vecAllPoints[3 * n + 2]);
			}
			for (int i = 3 * (n + 1); i < 4 * (n + 1); i++)
			{
				m_vecAllNormals.push_back(m_vecAllPoints[i] - m_vecAllPoints[4 * n + 3]);
			}
		}
	};
}