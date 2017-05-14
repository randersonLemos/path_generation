#ifndef OLS_H
#define OLS_H

#include <stdio.h>
#include <gsl/gsl_fit.h>

extern "C" int ols_2Dline(double **data, int ndata, double *model);

#endif /* OLS */
