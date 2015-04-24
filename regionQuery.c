//#include "dbScan.cpp"

int dbScan::regionQuery(int start, int index)
{
	int i, j, count = 0;
	double distance, temp;
	
	for(i = 0; i < n_pts; i++)
	{
		if(i != index)
		{
			distance = 0;
		
			for(j = 0; j < dim; j++)
			{
				temp = data[i][j] - data[index][j];
			
				distance += temp * temp;
			}
		
			if(distance <= eps)
			{
				neigh_points[start+count] = i;
			
				count++;
			}
		}
	}
	
	return count;
}
