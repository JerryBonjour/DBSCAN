//#include "dbScan.cpp"

void dbScan::loadData()
{
	/* allocate memory for arrays */
	
	visited = (int*)calloc(n_pts, sizeof(int));
	
	neigh_points = (int*)calloc(n_pts*n_pts, sizeof(int));
	
	clusters = (int*)calloc(n_pts, sizeof(int));
	
	/* ******** legacy code (initially read data from file) ********
	FILE *fp;
	fp = fopen(file_name, "r");
	
	for(i = 0; i < n_pts; i++)
	{
		for(j = 0; j < dim; j++)
		{
			fscanf(fp, "%d", &data[i][j]);
		}
	}
	
	fclose(fp);
	*********************************************** */
}
