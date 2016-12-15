#include <limits>
#include <list>

#include <Eigen/Dense>
#include <glog/logging.h>

#include "nanoflann/nanoflann.hpp"

/// \brief Custom result set class that only returns number of neighbors and their indices.
///        Default version also returns distances which we don't need.
template <typename DistanceType, typename IndexType = size_t>
class RadiusCountResultSet {
public:
  const DistanceType radius_;

  std::vector<IndexType>& m_indices_;

  inline RadiusCountResultSet(DistanceType radius, std::vector<IndexType>& m_indices) :
      radius_(radius), m_indices_(m_indices) {
    init();
  }

  inline ~RadiusCountResultSet() {}

  inline void init() { clear(); }
  inline void clear() { m_indices_.clear(); }

  inline size_t size() const { return m_indices_.size(); }

  inline bool full() const { return true; }

  inline void addPoint(DistanceType dist, IndexType index) {
    if (dist < radius_) m_indices_.push_back(index);
  }

  inline DistanceType worstDist() const { return radius_; }

};

/// \brief Base class for DBSCAN algorithm
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

    inline void clear() {
      count = 0u;
      new_neighbors.clear();
    }
  };

  void expandCluster(std::list<IndType>& neighbors, const int& cur_cluster) {
    for (auto&& cur_neighbor : neighbors) {
      if (!queried_[cur_neighbor]) {
        queried_[cur_neighbor] = true;

        regionQuery(cur_neighbor);
        if (query_.count >= min_pts_) {
          setCluster(query_.new_neighbors, cur_cluster);
          neighbors.splice(neighbors.end(), query_.new_neighbors);
        }
      }
    }
  }

  inline void regionQuery(const IndType& query_index) {
    result_set_.clear();
    query_.clear();
    // Extract query point to nanoflann format.
    for (IndType i = 0; i < dim_; ++i) query_pt_[i] = points_(query_index, i);
    // Perform radius search.
    kd_tree_.index->radiusSearchCustomCallback(&query_pt_[0], result_set_, params_);
    for (auto&& cur_idx: ret_matches_) {
      if (cur_idx != query_index) {	// Self point is no match.
        ++query_.count;
        if ((*clusters_)(cur_idx) == 0) query_.new_neighbors.push_back(cur_idx);
      }
    }
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

  // Variables to avoid reallocation during exectution.
  std::vector<Scalar> query_pt_;
  std::vector<IndType> ret_matches_;
  RadiusCountResultSet<Scalar, IndType> result_set_;
  QueryReturn query_;
  std::list<IndType> expansion_queue_;

  // KD Tree.
  KDTreeType kd_tree_;

public:
  /// \brief Create DBSCAN cluster element.
  /// \param points Data matrix of clustered point with size n_points x dim.
  /// \param eps Maximum distance for a point to be considered part of a cluster.
  /// \param min_pts Minimum number of points in vincinity to be considered a base point.
  DbscanBase(MatRefType points, const Scalar& eps, const unsigned int& min_pts) :
      points_(points), n_pts_(points.rows()), dim_(points.cols()), eps_(eps*eps),
      min_pts_(min_pts), query_pt_(dim_), result_set_(eps_, ret_matches_),
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

        regionQuery(i);

        if(query_.count >= min_pts_) {
          expansion_queue_.swap(query_.new_neighbors);
           (*clusters_)(i) = cur_cluster;
          setCluster(expansion_queue_, cur_cluster);
          expandCluster(expansion_queue_, cur_cluster);

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
