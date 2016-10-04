#include "dbscan/dbscan.h"

#include <glog/logging.h>

Dbscan::Dbscan(const Eigen::MatrixXd& points, const double eps, const unsigned int min_pts) :
    points_(points), eps_(eps*eps), min_pts_(min_pts) {
  n_pts_ = points.rows();
  dim_ = points.cols();

  queried_.resize(n_pts_, false);

  kd_tree_ = std::unique_ptr<my_kd_tree_t>(
        new my_kd_tree_t(dim_, points_, 20 /* max leafs */ ));
  kd_tree_->index->buildIndex();
}

void Dbscan::cluster(Eigen::VectorXi* clusters) {
  CHECK_NOTNULL(clusters);
  (*clusters) = Eigen::VectorXi::Zero(n_pts_);
  clusters_ = clusters;

  unsigned int cur_cluster = 1;

  for(unsigned int i = 0; i < n_pts_; i++) {
    if(!queried_[i]) {
      queried_[i] = true;

      QueryReturn output = regionQuery(i);

      if(output.count >= min_pts_) {
        (*clusters_)(i) = cur_cluster;
        setCluster(output.new_neighbors, cur_cluster);
        expandCluster(output.new_neighbors, cur_cluster);

        cur_cluster++;
      }
    }
  }
  clusters_ = NULL;
}

void Dbscan::expandCluster(std::list<unsigned int>& neighbors, const unsigned int& cur_cluster) {
  for (auto&& cur_neighbor : neighbors) {
    if (!queried_[cur_neighbor]) {
      queried_[cur_neighbor] = true;

      QueryReturn query = regionQuery(cur_neighbor);
      if (query.count >= min_pts_) {
        setCluster(query.new_neighbors, cur_cluster);
        neighbors.splice(neighbors.end(), query.new_neighbors);
      }
    }
  }
}

void Dbscan::setCluster(const std::list<unsigned int>& indices, const unsigned int& cur_cluster) {
  for (auto&& index : indices) {
    (*clusters_)(index) = cur_cluster;
  }
}

Dbscan::QueryReturn Dbscan::regionQuery(const unsigned int& query_index) {
  std::vector<std::pair<long int, double> > ret_matches;
  nanoflann::SearchParams params;

  QueryReturn output;
  // Extract query point to nanoflann format.
  std::vector<double> query_pt(dim_);
  for (unsigned int i = 0; i < dim_; ++i) query_pt[i] = points_(query_index, i);
  // Perform radius search.
  const unsigned int n_matches = kd_tree_->index->radiusSearch(&query_pt[0], eps_, ret_matches, params);
  for (unsigned int i = 0; i < n_matches; i++) {
    const int cur_idx = ret_matches[i].first;
    if (cur_idx != query_index) {	// Self point is no match.
      output.count++;
      if ((*clusters_)(cur_idx) == 0) output.new_neighbors.push_back(cur_idx);
    }
  }
  return output;
}

