#ifndef CFFTPLAN_H
#define CFFTPLAN_H

#include <complex>
#include <fftw3.h>
#include <stdlib.h>
	
/**
@author Tom Krajnik
*/

using namespace std;

class CFFTPlan
{
public:
  CFFTPlan();
  ~CFFTPlan();

  void prepare(unsigned int length);

	fftw_plan direct;
	fftw_plan inverse;
	double *probability;
	double *signal;
	fftw_complex *coeffs;	
	FILE *wisdom;
};

#endif
