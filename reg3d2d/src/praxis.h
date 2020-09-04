extern "C" {
#include "machine.h"

double praxis( double (*_fun)(double*,int),double _x[N],int _n);
void halt_praxis();

}
