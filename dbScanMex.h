#include<stdio.h>
#include<stdlib.h>

double **data;
int *clusters;
int *visited;
int *neigh_points;

int dim;
int n_pts;
double eps;
int min_pts;

#include"loadData.c"
#include"printData.c" 
#include"freeData.c" 
#include"expandCluster.c"
#include"dbScan.c"
#include"regionQuery.c"
