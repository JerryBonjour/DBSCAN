// This is an exampleof a custom data set class
using namespace std;

template <typename T>
struct PointCloud
{  
  struct Point
  {
    vector<T>  data;
  };

  std::vector<Point>  pts;
  int dim;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
  inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
  {
    T dist = 0;
    for (int i = 0; i < dim; i++) {
      const T d = p1[i] - pts[idx_p2].data[i];
      dist += d*d;
    }    
    return dist;
  }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate value, the
  //  "if/else's" are actually solved at compile time.
  inline T kdtree_get_pt(const size_t idx, int dim) const
  {    
    return pts[idx].data[dim];
  }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};