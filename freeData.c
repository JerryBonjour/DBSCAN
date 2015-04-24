void freeData()
{
  int i;
  for (i = 0; i < n_pts; i++) {
    free(data[i]);
  }
  free(data);
  free(clusters);
  free(visited);
  free(neigh_points);
}
