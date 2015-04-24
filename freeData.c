//#include "dbScan.cpp"

void dbScan::freeData()
{
  free(clusters);
  free(visited);
  free(neigh_points);
}
