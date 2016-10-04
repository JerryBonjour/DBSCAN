#include <list>
#include <memory>

#include <Eigen/Dense>

#include "nanoflann/nanoflann.hpp"

class Dbscan {

  typedef nanoflann::KDTreeEigenMatrixAdaptor< Eigen::MatrixXd > my_kd_tree_t;

  struct QueryReturn {
    QueryReturn() : count(0) {}

    unsigned int count;
    std::list<unsigned int> new_neighbors;
  };

  void loadData();

  void expandCluster(std::list<unsigned int>& neighbors, const unsigned int& cur_cluster);

  QueryReturn regionQuery(const unsigned int& query_index);

  void setCluster(const std::list<unsigned int>& indices, const unsigned int& cur_cluster);

  Eigen::VectorXi* clusters_;
  std::vector<bool> queried_;

  // Data variables.
  const Eigen::MatrixXd& points_;
  unsigned int n_pts_;
  unsigned int dim_;

  // Clustering parameter
  const double eps_;
  const unsigned int min_pts_;

  std::unique_ptr<my_kd_tree_t> kd_tree_;

public:
  /// \brief Create DBSCAN cluster element.
  /// \param points Data matrix of clustered point with size n_points x dim.
  /// \param eps Maximum distance for a point to be considered part of a cluster.
  /// \param min_pts Minimum number of points in vincinity to be considered a base point.
  Dbscan(const Eigen::MatrixXd& points, const double& eps, const unsigned int& min_pts);

  /// \brief Perform clustering and return cluster index for each point. 0 indices are outliers.
  void cluster(Eigen::VectorXi* clusters);
};
