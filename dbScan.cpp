#include "include/RStarTree.h"
// #include "loadData.c"
// #include "freeData.c" 
// #include "regionQuery.c"
// #include "expandCluster.c"

template<size_t dimension>
class dbScan {
 public:
	dbScan(const int n_pts, const int eps, const int min_pts, double * ptr) 
	: dim(dimension) {
		this->n_pts = n_pts;
		this->ptr = ptr;
		this->eps = eps; 
		this->min_pts = min_pts;
		loadData();
		double * pointer = ptr;
		
  	// Build tree
  	int i, j;
  	BBox bb;
  	for(i = 0; i < n_pts; i++) {
  	  for ( j = 0; j < dimension; j++) {
  	    const double temp_val = *(pointer+j*n_pts);
  	    bb.edges[j].first = temp_val;
  	    bb.edges[j].second = temp_val;
  	  }
  	  data.Insert(i,bb);
  	  pointer++;
  	}
	}

	~dbScan() {
		freeData();
	}

	double * cluster() {
	  
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
	  
	}

 private:
 	void loadData(){
	/* allocate memory for arrays */
	
	visited = (int*)calloc(n_pts, sizeof(int));
	
	neigh_points = (int*)calloc(n_pts*n_pts, sizeof(int));
	
	clusters = (int*)calloc(n_pts, sizeof(int));
	
	}
 	void freeData()
 	{
  free(clusters);
  free(visited);
  free(neigh_points);
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
	int i, j, count = 0;
	double distance, temp;
	double * pointer = ptr;
	BBox bound;
	for (i = 0; i < n_pts; i++) {
		if (i != index) {
			for ( j = 0; j < dimension; j++) {
  	    const double temp_val = *(pointer+j*n_pts);
  	    bound.edges[j].first = temp_val;
  	    bound.edges[j].second = temp_val;
  	  }
			Visitor visitor = data.Query(RTree::AcceptEnclosing(bound), Visitor());
		}
		pointer++;

	}
	
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
	
	int n_pts;
	double eps;
	int min_pts;	
	const int dim;
	double * ptr;
	typedef RStarTree<int, dimension, 32, 64> RTree;
	typedef typename RTree::BoundingBox BBox;
	typedef typename RTree::Leaf BLeaf;
	RTree data;

	struct Visitor {
		int count;
		bool ContinueVisiting;
		
		Visitor() : count(0), ContinueVisiting(true) {};
		
		void operator()(const BLeaf * const leaf) 
		{
			count++;
		}
	};
};
