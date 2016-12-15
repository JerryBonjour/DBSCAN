#include <limits>
#include <list>

#include <Eigen/Dense>
#include <glog/logging.h>

#include "nanoflann/nanoflann.hpp"

template <typename Derived, class Distance = nanoflann::metric_L2_Simple>
class DbscanBase {

  typedef Eigen::DenseBase<Derived> MatType;
  typedef const MatType& MatRefType;
  typedef nanoflann::KDTreeEigenMatrixAdaptor< MatType, MatType::ColsAtCompileTime, Distance > KDTreeType;
  typedef typename KDTreeType::IndexType IndType;
  typedef typename KDTreeType::num_t Scalar;

  struct QueryReturn {
    QueryReturn() : count(0u) {}

    size_t count;
    std::list<IndType> new_neighbors;
  };

  void loadData();

  void expandCluster(std::list<IndType>& neighbors, const int& cur_cluster) {
    QueryReturn query;
    for (auto&& cur_neighbor : neighbors) {
      if (!queried_[cur_neighbor]) {
        queried_[cur_neighbor] = true;

        query = regionQuery(cur_neighbor);
        if (query.count >= min_pts_) {
          setCluster(query.new_neighbors, cur_cluster);
          neighbors.splice(neighbors.end(), query.new_neighbors);
        }
      }
    }
  }

  QueryReturn regionQuery(const IndType& query_index) {
    std::vector<std::pair<IndType, Scalar> > ret_matches;

    QueryReturn output;
    // Extract query point to nanoflann format.
    std::vector<Scalar> query_pt(dim_);
    for (IndType i = 0; i < dim_; ++i) query_pt[i] = points_(query_index, i);
    // Perform radius search.
    const size_t n_matches =
        kd_tree_.index->radiusSearch(&query_pt[0], eps_, ret_matches, params_);
    for (size_t i = 0u; i < n_matches; ++i) {
      const IndType cur_idx = ret_matches[i].first;
      if (cur_idx != query_index) {	// Self point is no match.
        ++output.count;
        if ((*clusters_)(cur_idx) == 0) output.new_neighbors.push_back(cur_idx);
      }
    }
    return output;
  }

  void setCluster(const std::list<IndType>& indices, const int& cur_cluster) {
    for (auto&& index : indices) {
      (*clusters_)(index) = cur_cluster;
    }
  }

  Eigen::VectorXi* clusters_;
  std::vector<bool> queried_;

  // Data variables.
  MatRefType points_;
  const IndType n_pts_;
  const IndType dim_;

  // Clustering parameter
  const Scalar eps_;
  const size_t min_pts_;
  nanoflann::SearchParams params_;

  KDTreeType kd_tree_;

public:
  /// \brief Create DBSCAN cluster element.
  /// \param points Data matrix of clustered point with size n_points x dim.
  /// \param eps Maximum distance for a point to be considered part of a cluster.
  /// \param min_pts Minimum number of points in vincinity to be considered a base point.
  DbscanBase(MatRefType points, const Scalar& eps, const unsigned int& min_pts) :
      points_(points), n_pts_(points.rows()), dim_(points.cols()), eps_(eps*eps), min_pts_(min_pts),
      kd_tree_(dim_, points_, 20 /* max leafs */ ) {
    params_.sorted = false;
  }

  /// \brief Perform clustering and return cluster index for each point. 0 indices are outliers.
  void cluster(Eigen::VectorXi* clusters) {
    CHECK_NOTNULL(clusters);
    (*clusters) = Eigen::VectorXi::Zero(n_pts_);
    clusters_ = clusters;
    queried_.assign(n_pts_, false);

    int cur_cluster = 1;

    for (size_t i = 0u; i < n_pts_; ++i) {
      if(!queried_[i]) {
        queried_[i] = true;

        QueryReturn query = regionQuery(i);

        if(query.count >= min_pts_) {
           (*clusters_)(i) = cur_cluster;
          setCluster(query.new_neighbors, cur_cluster);
          expandCluster(query.new_neighbors, cur_cluster);

          ++cur_cluster;
          CHECK_NE(cur_cluster, std::numeric_limits<int>::max())
              << "Too many clusters! Number of clusters equal std::numeric_limits<int>::max()";
        }
      }
    }
    clusters_ = NULL;
  }
};

namespace Dbscan{

/// \brief Shorthand to call DBSCAN cluster algorithm.
/// \param points Data matrix of clustered point with size n_points x dim.
/// \param eps Maximum distance for a point to be considered part of a cluster.
/// \param min_pts Minimum number of points in vincinity to be considered a base point.
/// \param clusters Pointer to vector for clustering output.
template<typename Derived, typename Distance = nanoflann::metric_L2_Simple>
void Cluster(const Derived& points,
             const typename Derived::Scalar& eps,
             const unsigned int& min_pts,
             Eigen::VectorXi* clusters) {
  DbscanBase<Derived, Distance> clusterer(points, eps, min_pts);
  clusterer.cluster(clusters);
}

}
