void dbScan()
{
	int next_cluster = 1, i, j, num_npoints;
	
	for(i = 0; i < n_pts; i++)
	{
		if (i%100 == 0) {
			mexPrintf("At iteration %d\n", i);
			mexCallMATLAB(0, NULL, 0, NULL, "drawnow");
		}
		if(!visited[i])
		{
			visited[i] = 1;
			
			num_npoints = regionQuery(0, i);
			
			if(num_npoints > min_pts)
			{
				expandCluster(next_cluster, num_npoints, i);
				
				next_cluster++;
			}
		}
	}
}
