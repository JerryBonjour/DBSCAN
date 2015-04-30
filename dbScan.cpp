#include "include/nanoflann.hpp"
#include "nanoflann_helper.hpp"
#include <iostream>
// #include "loadData.c"
// #include "freeData.c" 
// #include "regionQuery.c"
// #include "expandCluster.c"

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
	  
		int next_cluster = 1, i, j, num_npoints;
		
		for(i = 0; i < n_pts; i++) {
			if(!visited[i]) {
				visited[i] = 1;
				
				num_npoints = regionQuery(0, i);
				
				if(num_npoints > min_pts) {
					expandCluster(next_cluster, num_npoints, i);
					
					next_cluster++;
				}
			}
		}
		return clusters;
	  
	}

 private:
 	void loadData(){
		/* allocate memory for arrays */
		
		visited = (int*)calloc(n_pts, sizeof(int));
		
		neigh_points = (int*)calloc(n_pts*n_pts, sizeof(int));
		
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
		clusters[index] = cluster_no;
		
		int i, count = 0;
		
		for(i = 0; i < num_npoints; i++) {
			if(!visited[neigh_points[i]]) {
				visited[neigh_points[i]] = 1;
				
				count = regionQuery(num_npoints, neigh_points[i]);
				
				if(count >= min_pts) {
					num_npoints += count;
				}
			}
			
			if(!clusters[neigh_points[i]]) {
				clusters[neigh_points[i]] = cluster_no;
				/* printf("Hi\n"); */
			}
		}
	}

 	int regionQuery(int start, int index)
 	{
 		vector<std::pair<size_t,T> > ret_matches;
 		SearchParams params;

 		int count = 0;
 		T * query_pt = data.pts[index].data;
		const size_t nMatches = tree->radiusSearch(&query_pt[0],eps, ret_matches, params);	
 		for (size_t i = 0; i < nMatches; i++) {
 			int cur_idx = ret_matches[i].first;	// get index of current point
 			if (cur_idx != index) {
 				neigh_points[start+count] = cur_idx;
 				count++;
 			}
 		}
 		// int i, j, count = 0;
		// double distance, temp;
		//for(i = 0; i < n_pts; i++)
		//{
		//	if(i != index)
		//	{
		//		distance = 0;
		//	
		//		for(j = 0; j < dim; j++)
		//		{
		//			temp = data[i][j] - data[index][j];
		//		
		//			distance += temp * temp;
		//		}
		//	
		//		if(distance <= eps)
		//		{
		//			neigh_points[start+count] = i;
		//		
		//			count++;
		//		}
		//	}
		//}
		
		return count;
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
