#ifndef DBSCAN_HPP
#define DBSCAN_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace lidar_detector
{

struct Point3
{
	float x;
	float y;
	float z;
};

class DBSCAN
{
public:
	// eps: neighborhood radius (meters). min_pts: minimum points to form a core point.
	DBSCAN(double eps = 0.5, int min_pts = 5)
	: eps_(eps), min_pts_(min_pts), last_cluster_id_(0)
	{
		eps2_ = eps_ * eps_;
	}

	void setEps(double eps)
	{
		eps_ = eps;
		eps2_ = eps_ * eps_;
	}

	void setMinPts(int m) { min_pts_ = m; }

	// Run DBSCAN on a vector of Point3. Returns labels: -1 = noise, >=1 cluster id.
	std::vector<int> run(const std::vector<Point3> &points)
	{
		points_ = points; // store for centroid computation
		const int n = static_cast<int>(points_.size());
		labels_.assign(n, UNVISITED);
		last_cluster_id_ = 0;

		for (int i = 0; i < n; ++i)
		{
			if (labels_[i] != UNVISITED)
				continue;

			std::vector<int> neighbors = regionQuery(i);
			if (static_cast<int>(neighbors.size()) < min_pts_)
			{
				labels_[i] = NOISE;
			}
			else
			{
				++last_cluster_id_;
				expandCluster(i, neighbors, last_cluster_id_);
			}
		}

		return labels_;
	}

	// Run DBSCAN directly on a sensor_msgs::msg::PointCloud2 and return labels.
	std::vector<int> run(const sensor_msgs::msg::PointCloud2 &cloud)
	{
		std::vector<Point3> pts;
		sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
		sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
		sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

		for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
		{
			float x = *it_x;
			float y = *it_y;
			float z = *it_z;
			if (std::isnan(x) || std::isnan(y) || std::isnan(z))
				continue;
			pts.push_back({x, y, z});
		}

		return run(pts);
	}

	// Compute centroids for discovered clusters. Returns centroids ordered by cluster id (1..K).
	std::vector<Point3> clusterCentroids() const
	{
		std::unordered_map<int, std::pair<Point3, int>> sums; // cluster_id -> (sum, count)
		const int n = static_cast<int>(labels_.size());
		for (int i = 0; i < n; ++i)
		{
			int lab = labels_[i];
			if (lab <= 0)
				continue; // skip noise and unvisited (shouldn't be unvisited after run)

			auto &entry = sums[lab];
			entry.first.x += points_[i].x;
			entry.first.y += points_[i].y;
			entry.first.z += points_[i].z;
			entry.second += 1;
		}

		std::vector<Point3> centroids;
		if (sums.empty())
			return centroids;

		// clusters may have ids 1..last_cluster_id_
		centroids.resize(last_cluster_id_);
		for (const auto &kv : sums)
		{
			int cid = kv.first; // cluster id
			const Point3 &sum = kv.second.first;
			int cnt = kv.second.second;
			if (cid >= 1 && cid <= last_cluster_id_ && cnt > 0)
			{
				centroids[cid - 1].x = sum.x / cnt;
				centroids[cid - 1].y = sum.y / cnt;
				centroids[cid - 1].z = sum.z / cnt;
			}
		}

		return centroids;
	}

	int getNumClusters() const { return last_cluster_id_; }

private:
	static constexpr int UNVISITED = 0;
	static constexpr int NOISE = -1;

	double eps_;
	double eps2_;
	int min_pts_;
	int last_cluster_id_;

	std::vector<Point3> points_;
	std::vector<int> labels_;

	inline double dist2(const Point3 &a, const Point3 &b) const
	{
		const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
		const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
		const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
		return dx * dx + dy * dy + dz * dz;
	}

	std::vector<int> regionQuery(int idx) const
	{
		std::vector<int> ret;
		const int n = static_cast<int>(points_.size());
		const Point3 &p = points_[idx];
		for (int i = 0; i < n; ++i)
		{
			if (i == idx)
			{
				ret.push_back(i);
				continue;
			}
			if (dist2(p, points_[i]) <= eps2_)
				ret.push_back(i);
		}
		return ret;
	}

	void expandCluster(int idx, std::vector<int> &neighbors, int cluster_id)
	{
		labels_[idx] = cluster_id;
		std::queue<int> q;
		for (int nb : neighbors)
		{
			if (labels_[nb] == UNVISITED)
			{
				labels_[nb] = cluster_id;
				q.push(nb);
			}
			else if (labels_[nb] == NOISE)
			{
				labels_[nb] = cluster_id; // border point becomes part of cluster
			}
		}

		while (!q.empty())
		{
			int current = q.front();
			q.pop();
			std::vector<int> result = regionQuery(current);
			if (static_cast<int>(result.size()) >= min_pts_)
			{
				for (int r : result)
				{
					if (labels_[r] == UNVISITED)
					{
						labels_[r] = cluster_id;
						q.push(r);
					}
					else if (labels_[r] == NOISE)
					{
						labels_[r] = cluster_id;
					}
				}
			}
		}
	}
};

} // namespace lidar_detector

#endif // DBSCAN_HPP