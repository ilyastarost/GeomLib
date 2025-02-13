#pragma once
#include "ThreadPool.h"
#include "Cylinder.h"
#include "Segment.h"
#include "Matrix.h"
#include "Plane.h"
#include "Ray.h"
#include <climits>
#include <vector>
#include <thread>
#include <set>

#include "Timer.h"

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

		Vector<T> NormalToCoords(const Point<T>& a, const Point<T>& b, const Point<T>& c) const
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

		bool IntersectsTriangle(int ind, const Ray<T>& ray, Point<T>& pt) const
		{
			auto check = Plane<T>(m_vecAllPoints[m_vecTriangles[ind].ind[0]], NormalToTriangle(ind)).FindIntersections(ray);
			if (check.size() == 0) 
				return false;
			Point<T> p = check[0];
			Vector<T> n1 = NormalToCoords(m_vecAllPoints[m_vecTriangles[ind].ind[0]], m_vecAllPoints[m_vecTriangles[ind].ind[1]], p);
			Vector<T> n2 = NormalToCoords(m_vecAllPoints[m_vecTriangles[ind].ind[1]], m_vecAllPoints[m_vecTriangles[ind].ind[2]], p);
			Vector<T> n3 = NormalToCoords(m_vecAllPoints[m_vecTriangles[ind].ind[2]], m_vecAllPoints[m_vecTriangles[ind].ind[0]], p);

			if (!Epsilon::IsZero(n1.DotProduct(n2) - 1) || !Epsilon::IsZero(n1.DotProduct(n3) - 1)) return false;
			pt = p;
			return true;
		}

		bool FindIntersection(const Ray<T>& ray, Point<T>& pt, int& ind, int left = 0, int right = INT_MAX) const
		{
			START_AUTO_TIMER(parallel3);
			std::cout << left << " " << right << std::endl;

			Point<T> ans(DBL_MAX, DBL_MAX, DBL_MAX), cur(DBL_MAX, DBL_MAX, DBL_MAX);
			T dist = DBL_MAX;
			int pos = -1, lim = std::min(right, (int)m_vecTriangles.size());
			for (int i = left; i < lim; i++)
			{
				if (IntersectsTriangle(i, ray, cur))
				{
					T newDist = cur.DistancePow2(ray.Start());
					if (newDist < dist) {
						ans = cur;
						pos = i;
						dist = newDist;
					}
				}
			}
			pt = ans;
			ind = pos;
			if (dist == INT_MAX)
				return false;
			return true;
		}

	private:
		class TriangleTask : public ThreadTask
		{
		private:
			Ray<T> ray;
			Point<T> pt;
			Point<T>& ans;
			int ind;
			int& pos;
			int left, right;
			T& dist;
			const TessModel* parent;

		public:
			TriangleTask(const Ray<T>& _ray, Point<T>& _ans, int& _pos, T& _dist, int _left, int _right, const TessModel* par) : ans(_ans), pos(_pos), dist(_dist)
			{
				ray = _ray;
				left = _left;
				right = _right;
				parent = par;
			}
			void ToDo() override
			{
				parent->FindIntersection(ray, pt, ind, left, right);
				T newDist = pt.DistancePow2(ray.Start());
				ans = pt;
				pos = ind;
				dist = newDist;
			}
		};

	public:
		bool FindIntersectionParallel(const Ray<T>& ray, Point<T>& pt, int& ind, ThreadPool& tp) const
		{
			int num = 4 * std::thread::hardware_concurrency();
			std::vector<Point<T>> res(num);
			std::vector<int> tr(num);
			std::vector<T> dists(num);
			Point<T> ans(DBL_MAX, DBL_MAX, DBL_MAX), cur(DBL_MAX, DBL_MAX, DBL_MAX);
			T dist = DBL_MAX;
			int pos = -1, sz = (m_vecTriangles.size() + num - 1) / num;
			for (int i = 0; i < num; i++)
			{
				std::shared_ptr<ThreadTask> task(new TriangleTask(ray, res[i], tr[i], dists[i], i * sz, (i + 1) * sz, this));
				tp.AssignTask(task);
			}
			tp.WaitEnd();
			for (int i = 0; i < num; i++) {
				if (dists[i] < dist)
				{
					dist = dists[i];
					ans = res[i];
					ind = tr[i];
				}
			}
			pt = ans;
			ind = pos;
			return true;

			//int num = std::thread::hardware_concurrency();
			//std::vector<Point<T>> res(num);
			//std::vector<int> tr(num);
			//std::vector<std::thread> threads;
			//int sz = (m_vecTriangles.size() + num - 1) / num;
			//int mx = m_vecTriangles.size();

			//START_TIMER("parallel2");
			//for (int i = 0; i < num; i++)
			//{
			//	threads.emplace_back(
			//		[this,i,sz,mx,&res,&tr,&ray]{ 
			//			return this->FindIntersection(ray, res[i], tr[i], i * sz, std::min((i + 1) * sz, mx)); 
			//		});
			//	//threads[i].detach();
			//}
			//STOP_TIMER("parallel2");

			//Point<T> ans, cur;
			//T dist = DBL_MAX;
			//int pos = -1;
			//for (int i = 0; i < num; i++)
			//{
			//	threads[i].join();
			//	if (res[i].DistancePow2(ray.Start()) < dist) {
			//		ans = res[i];
			//		pos = tr[i];
			//	}
			//}
			//if (dist == DBL_MAX)
			//	return false;
			//pt = ans;
			//ind = pos;
			//return true;
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

		std::string ToString() const
		{
			std::stringstream out;
			out << "Model with points: " << std::endl;
			for (auto p : m_vecAllPoints)
				out << p.ToString() << std::endl;
			out << "    Normals: " << std::endl;
			for (auto v : m_vecAllNormals)
				out << v.ToString() << std::endl;
			out << "    Triangles: " << std::endl;
			for (auto t : m_vecTriangles)
				out << "         " << t.ind[0] << ' ' << t.ind[1] << ' ' << t.ind[2] << std::endl;
			out << "    Surfaces: " << std::endl;
			for (int i = 0; i < m_vecLastOfSurface.size(); i++)
				out << "         " << (i ? m_vecLastOfSurface[i - 1] + 1 : 0) << ' ' << m_vecLastOfSurface[i] << std::endl;
			return out.str();
		}

		void Serialize(std::ostream& out) const
		{
			int n = m_vecAllPoints.size();
			out.write((char*)&n, sizeof(int));
			for (auto& p : m_vecAllPoints)
				p.Serialize(out);
			n = m_vecAllNormals.size();
			out.write((char*)&n, sizeof(int));
			for (auto& v : m_vecAllNormals)
				v.Serialize(out);
			n = m_vecTriangles.size();
			out.write((char*)&n, sizeof(int));
			for (auto& t : m_vecTriangles) 
			{
				out.write((char*)&t.ind[0], sizeof(int));
				out.write((char*)&t.ind[1], sizeof(int));
				out.write((char*)&t.ind[2], sizeof(int));
			}
			n = m_vecLastOfSurface.size();
			out.write((char*)&n, sizeof(int));
			for (auto q : m_vecLastOfSurface) 
			{
				out.write((char*)&q, sizeof(int));
			}
		}

		void Deserialize(std::istream& in)
		{
			int n;
			in.read((char*)&n, sizeof(int));
			m_vecAllPoints.resize(n);
			for (auto& p : m_vecAllPoints)
				p.Deserialize(in);
			in.read((char*)&n, sizeof(int));
			m_vecAllNormals.resize(n);
			for (auto& v : m_vecAllNormals)
				v.Deserialize(in);
			in.read((char*)&n, sizeof(int));
			m_vecTriangles.resize(n);
			for (auto& t : m_vecTriangles)
			{
				in.read((char*)&t.ind[0], sizeof(int));
				in.read((char*)&t.ind[1], sizeof(int));
				in.read((char*)&t.ind[2], sizeof(int));
			}
			in.read((char*)&n, sizeof(int));
			m_vecLastOfSurface.resize(n);
			for (auto& q : m_vecLastOfSurface)
				in.read((char*)&q, sizeof(int));
		}
	};
}