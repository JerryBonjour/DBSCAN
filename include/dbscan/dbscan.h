#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include "nanoflann/nanoflann.hpp"

class Dbscan {

  typedef nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd > my_kd_tree_t;

  struct QueryReturn {
    int count;
    int count_not_visited;
  };

  void loadData();

  void freeData();

  void expandCluster(const unsigned int& cluster_no,
                     unsigned int num_npoints,
                     const unsigned int& index,
                     std::vector<bool>& visited);

  QueryReturn regionQuery(const unsigned int& start,
                          const unsigned int& index,
                          std::vector<bool>& visited);

  Eigen::VectorXi* clusters_;
  std::vector<bool> queried_;
  std::vector<int> neigh_points_;

  const Eigen::MatrixXd& points_;
  unsigned int n_pts_;
  unsigned int dim_;

  const double eps_;
  const unsigned int min_pts_;

  std::unique_ptr<my_kd_tree_t> kd_tree_;

public:
  /// \brief Create DBSCAN cluster element.
  /// \param points Data matrix of clustered point with size n_points x dim.
  /// \param eps Maximum distance for a point to be considered part of a cluster.
  /// \param min_pts Minimum number of points in vincinity to be considered a base point.
  Dbscan(const Eigen::MatrixXd& points, const double eps, const unsigned int min_pts);

  /// \brief Perform clustering and return cluster index for each point. 0 indices are outliers.
  void cluster(Eigen::VectorXi* clusters);
};
