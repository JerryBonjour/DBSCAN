/*
 * dbScan.c - example in MATLAB External Interfaces
 *
 * Performs DBSCAN clustering on the data matrix (x) 
 * with a certain distance threshold (eps)
 * and a certain minimum point number per cluster (min_pts)
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
#include "dbScanMex.h"

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

  n_pts = mxGetM(IN_x);  /* get number of points */
  dim = mxGetN(IN_x);  /* get number of dimensions */
  mexPrintf("Number of points %d\n", n_pts);
  eps = mxGetScalar(IN_eps);  /* get eps input */
  eps = eps*eps;  /* algorithm uses squared distance */
  min_pts = mxGetScalar(IN_min_pts);  /* get min_pts input */

  loadData();
  double *ptr = mxGetData(IN_x);
  mexPrintf("Test value: %f\n", *(ptr+1));
  mexCallMATLAB(0, NULL, 0, NULL, "drawnow");
  int i,j;
  for (j = 0 ; j < dim; j++) {    
    for (i = 0; i < n_pts; i++) {
      data[i][j] = *ptr;
      ptr++;
    }
  }
  mexPrintf("Test value: %f\n", data[1][0]);
  mexPrintf("Clustering...\n");
  mexCallMATLAB(0, NULL, 0, NULL, "drawnow");
  dbScan();
  mexPrintf("Done clustering!\n");
  printData();

}