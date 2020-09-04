#include <stdio.h>
#include <math.h>
#include "nrutil.h"
#include "linmin.h"
#define TINY 1.0e-25     	//A small number
#define ITMAX 200			//Maximum allowed iterations
int nGlobal=6;

/* Minimization fo a function func of n variables.
Inputs: initial starting point p[1..n]
		initial matrix xi[1..n][1..n], columns contain the initial set of directions(usually the n unit vectors)
		fractional tolerance in the function value ftol
Outputs: p is set to the best point found
		xi is the current direction set
		fret is the returned function value at p
		iter is the number of iterations taken

The routine "linmin" is used.     */

// float func(float p[]);
// 
// void powell(float p[], float **xi, int n, float ftol, int *iter, float *fret,float (*func)(float []));

void powell(float p[], float **xi, int n, float ftol, int *iter, float *fret,float (*func)(float []))
{
// printf("in powell\n");
	void linmin(float p[],float xi[],int n,float *fret,float (*func)(float []));
//	printf("here\n");
	int i,ibig,j;
	float del,fp,fptt,t,*pt,*ptt,*xit;

	pt=fvector(1,n);
	ptt=fvector(1,n);
	xit=fvector(1,n);
	*fret=(*func)(p);
	for (j=1;j<=n;j++) pt[j]=p[j];  // save the initial point
	for (*iter=1;;++(*iter)) {
		printf("iteration=%d, fret=%f\n",*iter,*fret);
		fp=(*fret);
		ibig=0;
		del=0.0;   					// will be the biggest function decrease.
		for(i=1;i<=n;i++) {			// in each iteration, loop over all directions in the set.
			for(j=1;j<=n;j++) xit[j]=xi[j][i];	// copy the direction
			fptt=(*fret);
			linmin(p,xit,n,fret,func);  // minimize long it,
			if (fptt-(*fret)>del) {		// and record it if it is the largest decrease so far.
				del=fptt-(*fret);
				ibig=i;
			}
		}
		if (2.0*(fp-(*fret))<=ftol*(fabs(fp)+fabs(*fret))+TINY) {
			free_vector(xit,1,n);		// Termination criterion
			free_vector(ptt,1,n);
			free_vector(pt,1,n);
			printf("Terminated\n");
			return;
		}
		if (*iter==ITMAX)
			nrerror("powell exceeding maximum iterations.");
		for (j=1;j<=n;j++) {			// Construct the extrapolated point and the average direction moved.
// save the old starting point
			ptt[j]=2.0*p[j]-pt[j];
			xit[j]=p[j]-pt[j];
			pt[j]=p[j];
		}
		fptt=(*func)(ptt);				// function value at extrapolated point
		if (fptt < fp) {
//t = 2.0*(fp-2.0*(*fret)+fptt)*SQR(fp-(*fret)-del)-del*SQR(fp-fptt);
			t = 2.0*(fp-2.0*(*fret)+fptt)*(fp-(*fret)-del)*(fp-(*fret)-del)-del*(fp-fptt)*(fp-fptt);
			if (t < 0.0) {
				linmin(p,xit,n,fret,func);	// move to the minimum of the new direction.
				for (j=1;j<=n;j++) {		// and save the new direction.
					xi[j][ibig]=xi[j][n];
					xi[j][n]=xit[j];
				}
			}
		}
	}
}     // back for another iteration
