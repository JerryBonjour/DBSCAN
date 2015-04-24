void printData()
{
	int i, j;
	
	FILE *fp;
	fp = fopen("prediction.txt", "w");
	
	
	for(i = 0; i < n_pts; i++)
	{
		fprintf(fp, "%d\n", clusters[i]); 
		/*printf("%d %d %d\n", visited[i], clusters[i], neigh_points[i]);*/
	}
}
