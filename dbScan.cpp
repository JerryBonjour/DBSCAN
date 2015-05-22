#include "include/nanoflann.hpp"
#include "nanoflann_helper.hpp"
#include <iostream>
//#include <mex.h>	//use for console output when debugging

using namespace std;
using namespace nanoflann;

template <typename T>
class dbScan {
 public:
	dbScan(const int n_pts, const int dim, const T eps, const int min_pts, double * ptr)
		: n_pts(n_pts), dim(dim), eps(eps), min_pts(min_pts) {
		this->ptr = ptr;
		loadData();
	}

	~dbScan() {
		freeData();
	}

	int * cluster() {
	  
		int next_cluster = 1, i, j, num_npoints;	//initialize cluster
		
		for(i = 0; i < n_pts; i++) {	//iterate over all points
			if(!visited[i]) {	//enter if not yet visited
				visited[i] = 1;	//has now been visited
				
				QueryReturn output = regionQuery(0, i);	//check neighbors
				num_npoints = output.count;	//number of neighbors
				
				if(num_npoints > min_pts) {	//if enough neighbors
					expandCluster(next_cluster, num_npoints, i);	//expand cluster
					
					next_cluster++;	//move to next cluster
				}
			}
		}
		return clusters;
	  
	}

 private:
 	struct QueryReturn {
 		int count;
 		int count_not_visited;
 	};

 	void loadData(){
		/* allocate memory for arrays */
		
		visited = (int*)calloc(n_pts, sizeof(int));
		
		neigh_points = (int*)calloc(n_pts, sizeof(int));
		
		clusters = (int*)calloc(n_pts, sizeof(int));

		data.pts.resize(n_pts);
		data.dim = dim;
		// write data to pointcloud
		for ( int i = 0; i < n_pts; i++) {
			T * temp = new T[dim];
			for (int j = 0; j < dim; j++) {
				temp[j] = *(ptr+j*n_pts);
			}
			data.pts[i].data = temp;	
			ptr++;
		}
		// Build tree
		tree = new my_kd_tree_t(dim, data, KDTreeSingleIndexAdaptorParams(20 /* max leaf */) );
		tree->buildIndex();
	}

 	void freeData()
 	{
	  free(clusters);
	  free(visited);
	  free(neigh_points);
	  for (int i = 0; i < n_pts; i++) {
  		delete data.pts[i].data;
	  }
	}

 	void expandCluster(int cluster_no, int num_npoints, int index)
 	{
		clusters[index] = cluster_no;	//assign current point to cluster
		
		int i, count = 0;
		
		for(i = 0; i < num_npoints; i++) {	//loop until all points belonging to cluster visited

			QueryReturn output = regionQuery(num_npoints, neigh_points[i]);	//query with point
			count = output.count;	//get count of neighbors
			
			if(count >= min_pts) {	//if more than min_pts neighbors
				num_npoints += output.count_not_visited;	//add as basepoint
			}
			
			if(!clusters[neigh_points[i]]) {	//if neighbors don't have a class yet
				clusters[neigh_points[i]] = cluster_no;	//give it current class
			}
		}
	}

 	QueryReturn regionQuery(int start, int index)
 	{
 		vector<std::pair<size_t,T> > ret_matches;
 		SearchParams params;

 		QueryReturn output;
 		output.count = 0;
 		output.count_not_visited = 0;
 		T * query_pt = data.pts[index].data;	//point to query with
		const size_t nMatches = tree->radiusSearch(&query_pt[0],eps, ret_matches, params);	//perform radius search
 		for (size_t i = 0; i < nMatches; i++) {	//loop over all points inside eps radius
 			const int cur_idx = ret_matches[i].first;	// get index of current point
 			if (cur_idx != index) {	//self point is no match
 				output.count++;	//increment point counter
 				if (!visited[cur_idx]) {	//if point not yet visited
 					visited[cur_idx] = 1;	//marks as visited because it is now in queue
 					neigh_points[start+output.count_not_visited] = cur_idx;	//add to queue
 					output.count_not_visited++;	//increment counter
 				}
 			}
 		}
		
		return output;
	}

 	int *clusters;
	int *visited;
	int *neigh_points;
	
	const int n_pts;
	const T eps;
	const int min_pts;	
	const int dim;
	double * ptr;	
	PointCloud<T> data;

	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<T, PointCloud<T> > ,
		PointCloud<T>		
		> my_kd_tree_t;
	my_kd_tree_t * tree;

};
