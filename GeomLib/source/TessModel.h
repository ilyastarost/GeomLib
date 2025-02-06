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

	struct Node
	{
		int x;
		int y;
		int z;
		int ind;
	};

	bool comp(const Node& lhs, const Node& rhs)
	{
		if (lhs.x != rhs.x)
			return lhs.x < rhs.x;
		if (lhs.y != rhs.y)
			return lhs.y < rhs.y;
		if (lhs.z != rhs.z)
			return lhs.z < rhs.z;
		return lhs.ind < rhs.ind;
	}

	FLOATING(T)
	class TessModel
	{
	private:
		std::set<Node, decltype(&comp)> srtPoints;
		std::set<Node, decltype(&comp)> srtTriangles;
	protected:
		std::vector<Point<T>> m_vecAllPoints;
		std::vector<Triangle> m_vecTriangles;
		std::vector<int> m_vecLastOfSurface;

		void MergePointsAndTriangles(const std::vector<Point<T>>& pts, std::vector<Triangle> tr)
		{
			std::vector<int> mapping(pts.size());
			for (int i = 0; i < pts.size(); i++)
			{
				Node cur = { pts[i].X() / Epsilon::Eps(), pts[i].Y() / Epsilon::Eps(), pts[i].Z() / Epsilon::Eps(), -1 };
				int ind = GetIndexOfPoint(cur);
				mapping[i] = (ind == -1 ? m_vecAllPoints.size() : ind);
				cur.ind = m_vecAllPoints.size();
				if (ind == -1)
				{
					srtPoints.insert(cur);
					m_vecAllPoints.push_back(pts[i]);
				}
			}

			for (int i = 0; i < tr.size(); i++)
			{
				tr[i].ind[0] = mapping[tr[i].ind[0]];
				tr[i].ind[1] = mapping[tr[i].ind[1]];
				tr[i].ind[2] = mapping[tr[i].ind[2]];
				ReorderTriangle(tr[i]);
				Node cur = { tr[i].ind[0], tr[i].ind[1], tr[i].ind[2], m_vecTriangles.size() };
				srtTriangles.insert(cur);
				m_vecTriangles.push_back(tr[i]);
			}
		}

	public:
		TessModel()
		{
			srtPoints = std::set<Node, decltype(&comp)>(comp);
			srtTriangles = std::set<Node, decltype(&comp)>(comp);
		}

		int GetIndexOfPoint(const Node& pt) const
		{
			auto res = srtPoints.lower_bound(pt);
			return (res != srtPoints.end() && res->x == pt.x && res->y == pt.y && res->z == pt.z ? res->ind : -1);
		}

		int GetIndexOfTriangle(Triangle tr) const
		{
			ReorderTriangle(tr);
			auto res = srtTriangles.lower_bound(tr);
			return (res != srtTriangles.end() && res->x == tr.x && res->y == tr.y && res->z == tr.z ? res->ind : -1);
		}

		Vector<T> NormalToTriangle(int ind) const
		{
			return (m_vecAllPoints[m_vecTriangles[ind].ind[1]] - m_vecAllPoints[m_vecTriangles[ind].ind[0]]).CrossProduct(
					m_vecAllPoints[m_vecTriangles[ind].ind[2]] - m_vecAllPoints[m_vecTriangles[ind].ind[0]]).Normalize();
		}

		static void ReorderTriangle(Triangle& tr)
		{
			int mn = 1e9, pos = -1;
			for (int i = 0; i < 3; i++)
			{
				if (tr.ind[i] < mn) mn = tr.ind[i], pos = i;
			}
			int res[3];
			for (int i = 0; i < 3; i++)
			{
				res[i] = tr.ind[(pos + i) % 3];
			}
			memcpy(tr.ind, res, 3 * sizeof(int));
		}

		void MergeModels(const TessModel<T>& model) 
		{
			MergePointsAndTriangles(model.m_vecAllPoints, model.m_vecTriangles);
			int old = m_vecLastOfSurface.back();
			for (int i = 0; i < model.m_vecLastOfSurface.size(); i++)
			{
				m_vecLastOfSurface.push_back(old + model.m_vecLastOfSurface[i]);
			}
		}

		void AddSurface(const std::vector<Point<T>>& pts, const std::vector<Triangle>& tr)
		{
			MergePointsAndTriangles(pts, tr);
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
	};
}