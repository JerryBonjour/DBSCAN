void loadData()
{
	/* allocate memory for arrays */
	data = (double**)calloc(n_pts, sizeof(double*));
	
	visited = (int*)calloc(n_pts, sizeof(int));
	
	neigh_points = (int*)calloc(n_pts*n_pts, sizeof(int));
	
	clusters = (int*)calloc(n_pts, sizeof(int));
	
	int i, j;
	
	for(i = 0; i < n_pts; i++)
	{
		data[i] = (double*)calloc(dim, sizeof(double));
	}
	
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
