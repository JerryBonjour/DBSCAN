#include "dbscan/dbscan.h"

#include <glog/logging.h>
#include <iostream>

Dbscan::Dbscan(const Eigen::MatrixXd& points, const double eps, const unsigned int min_pts) :
    points_(points), eps_(eps*eps), min_pts_(min_pts) {
  n_pts_ = points.rows();
  dim_ = points.cols();
  loadData();
}

void Dbscan::cluster(Eigen::VectorXi* clusters) {
  CHECK_NOTNULL(clusters)->resize(n_pts_);
  (*clusters) = Eigen::VectorXi::Zero(n_pts_);
  clusters_ = clusters;

  int next_cluster = 1;	//initialize cluster

  for(unsigned int i = 0; i < n_pts_; i++) {
    if(!queried_[i]) {
      std::vector<bool> visited(n_pts_, false);
      queried_[i] = true;

      QueryReturn output = regionQuery(0, i, visited);	//check neighbors
      const unsigned int num_npoints = output.count;	//number of neighbors

      if(num_npoints > min_pts_) {	//if enough neighbors
        expandCluster(next_cluster, num_npoints, i, visited);	//expand cluster

        next_cluster++;	//move to next cluster
      }
    }
  }
  clusters_ = NULL;
}

void Dbscan::loadData() {
  queried_.resize(n_pts_, false);
  neigh_points_.resize(n_pts_, -1);

  kd_tree_ = std::unique_ptr<my_kd_tree_t>(
        new my_kd_tree_t(dim_, points_, 20 /* max leafs */ ));
  kd_tree_->index->buildIndex();
}

void Dbscan::expandCluster(const unsigned int& cluster_no,
                           unsigned int num_npoints,
                           const unsigned int& index,
                           std::vector<bool>& visited) {
  (*clusters_)(index) = cluster_no;	//assign current point to cluster

  for(unsigned int i = 0; i < num_npoints; i++) {	//loop until all points belonging to cluster visited

    const unsigned int cur_neighbor = neigh_points_[i];
    if(!(*clusters_)(cur_neighbor)) {	//if neighbors don't have a class yet
      (*clusters_)(cur_neighbor) = cluster_no;	//give it current class
    }

    if (!queried_[cur_neighbor]) {
      queried_[cur_neighbor] = true;
      QueryReturn output = regionQuery(num_npoints, cur_neighbor, visited);	//query with point

      if(output.count >= min_pts_) {	//if more than min_pts neighbors
        num_npoints += output.count_not_visited;	//add as basepoint
      }
    }
  }
}

Dbscan::QueryReturn Dbscan::regionQuery(const unsigned int& start,
                                        const unsigned int& index,
                                        std::vector<bool>& visited) {
  std::vector<std::pair<long int, double> > ret_matches;
  nanoflann::SearchParams params;

  QueryReturn output;
  output.count = 0;
  output.count_not_visited = 0;
  // Extract query point to nanoflann format.
  std::vector<double> query_pt(dim_);
  for (unsigned int i = 0; i < dim_; ++i) query_pt[i] = points_(index, i);
  // Perform radius search.
  const size_t n_matches = kd_tree_->index->radiusSearch(&query_pt[0], eps_, ret_matches, params);
  for (size_t i = 0; i < n_matches; i++) {
    const int cur_idx = ret_matches[i].first;
    if (cur_idx != index) {	// Self point is no match.
      output.count++;	// Increment point counter.
      if (n_matches > min_pts_) {
        if (!visited[cur_idx]) {	// If point not yet visited.
          visited[cur_idx] = true;
          neigh_points_[start+output.count_not_visited] = cur_idx;	// Add to queue.
          output.count_not_visited++;	// Increment counter.
        }
      }
    }
  }
  return output;
}

