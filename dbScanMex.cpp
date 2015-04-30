/*
 * Performs DBSCAN clustering
 * Input
 * - X: An M x N matrix where M is the number of points and N the dimensionality
 * - eps: A distance threshold 
 * - min_pts: Minimum number of points for a cluster
 * 
 * Output
 * - An M x 1 array of class labels. 0 labels don't belong to any cluster and can be considered outliers. 
 *
 * The calling syntax is:
 *
 *    classes = dbScan(x, eps, min_pts)
 *
 * This is a MEX-file for MATLAB.
*/

 #define IN_x         prhs[0]
 #define IN_eps       prhs[1]
 #define IN_min_pts   prhs[2]

 #define OUT          plhs[0]

#include <mex.h>
#include <stdio.h>
#include <stdlib.h>
#include "dbScan.cpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  if(nrhs!=3) {
      mexErrMsgIdAndTxt("MyToolbox:dbScanMex:nrhs", "Three inputs required.");
      mexErrMsgIdAndTxt("MyToolbox:dbScanMex:nlhs", "One output required.");
  }

  if( !mxIsDouble(IN_x) || 
     mxIsComplex(IN_x)) {
    mexErrMsgIdAndTxt("MyToolbox:dbScanMex:notDouble",
        "Input matrix must be type double.");
  }

  if( !mxIsDouble(IN_eps) || 
     mxIsComplex(IN_eps)) {
    mexErrMsgIdAndTxt("MyToolbox:dbScanMex:notDouble",
        "eps must be type double.");
  }

  if( !mxIsDouble(IN_min_pts) || 
     mxIsComplex(IN_min_pts)) {
    mexErrMsgIdAndTxt("MyToolbox:dbScanMex:notDouble",
        "min_pts must be type integer.");
  }

  mexPrintf("*********** Starting DBSCAN ***********\n");
  const int n_pts = mxGetM(IN_x);  /* get number of points */
  const int dim = mxGetN(IN_x);  /* get number of dimensions */
  mexPrintf("Input matrix is %d by %d\n", n_pts, dim);
  double eps = mxGetScalar(IN_eps);  /* get eps input */  
  eps = eps*eps;  // algorithm uses squared distance
  const int min_pts = mxGetScalar(IN_min_pts);  /* get min_pts input */
  mexPrintf("Parameters are: eps %f min_pts %d\n", eps, min_pts);

  int *clusters;
  mexPrintf("Building tree...\n");
  mexEvalString("drawnow;");
  //mexCallMATLAB(0, NULL, 0, NULL, "drawnow");

  dbScan<double> * clustering = new dbScan<double>(n_pts, dim, eps, min_pts, (double*)mxGetData(IN_x));
  //double clusters[n_pts];
  mexPrintf("Clustering...\n");
  //mexCallMATLAB(0, NULL, 0, NULL, "drawnow");
  mexEvalString("drawnow;");
  clusters = clustering->cluster();

  OUT = mxCreateDoubleMatrix(n_pts, 1, mxREAL); /* create output array */
  double *out_matrix = mxGetPr(OUT);
  for(int i = 0; i < n_pts; i++) {
    out_matrix[i] = clusters[i];  /* write to output array */
  }
  
  mexPrintf("Freeing memory...\n"); 
  delete clustering;

  mexPrintf("*********** DBSCAN finished ***********\n");
}