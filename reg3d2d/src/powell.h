#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <math.h>
#include "nrutil.h"
//#include "linmin.h"
void powell(float p[], float **xi, int n, float ftol, int *iter, float *fret,float (*func)(float []));

#ifdef __cplusplus
}
#endif