C++ implementation of the [DBSCAN](https://en.wikipedia.org/wiki/DBSCAN) clustering algorithm, originally adopted from https://github.com/siddharth-agrawal/DBSCAN but completely rewritten.
Uses [nanoflann](https://github.com/jlblancoc/nanoflann) for kd-tree radius search.

Inputs are:
* points: A M x N matrix where M is the number of points and N the dimensionality.
* eps: Radius around point to be considered a neighbor.
* min_pts: Minimum number of points in neighborhood to be considered a base point.

The output is:
* cluster: An M x 1 vector of class labels. 0 labels don't belong to any cluster and can be considered outliers. 
