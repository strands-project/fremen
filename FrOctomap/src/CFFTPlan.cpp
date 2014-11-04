#include "CFFTPlan.h"

using namespace std;
#define FFTW_TYPE FFTW_MEASURE

CFFTPlan::CFFTPlan()
{
	probability = signal = NULL;
	fftw_import_wisdom_from_filename("fftw.wisdom");
}

CFFTPlan::~CFFTPlan()
{
	free(signal);
	free(coeffs);
	free(probability);
	fftw_destroy_plan(direct);
	fftw_destroy_plan(inverse);
	fftw_cleanup();
}

void CFFTPlan::prepare(unsigned int signalLength)
{
	probability = (double *)fftw_malloc(signalLength*sizeof(double));
	signal = (double*)fftw_malloc(signalLength*sizeof(double));
	coeffs = (fftw_complex *)fftw_malloc((signalLength/2+1)*sizeof(fftw_complex));
	direct = fftw_plan_dft_r2c_1d(signalLength,signal,coeffs,FFTW_TYPE);
	inverse = fftw_plan_dft_c2r_1d(signalLength,coeffs,probability,FFTW_TYPE);
	fftw_export_wisdom_to_filename("fftw.wisdom");
}

