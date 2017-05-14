#include "ols.hpp"

int ols_2Dline(double **data, int ndata, double *model)
{
  int i = 0;
  int n = ndata;

  double x[n], y[n];

  for(i=0; i<n; ++i)
  {
    x[i] = data[i][0];
    y[i] = data[i][1];
  }
  double c0, c1, cov00, cov01, cov11, chisq;
  gsl_fit_linear(x, 1, y, 1, n, &c0, &c1, &cov00, &cov01, &cov11, &chisq);

  //printf("%f-%f\n",c0,c1);

  model[0] = -c1;
  model[1] = 1.0;
  model[2] = -c0;

  //printf("%f-%f-%f\n",model[0],model[1],model[2]);

  return 0;
}


