/******************************* SOURCE LICENSE *********************************
Copyright (c) 2015 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

// Begin header file, filter_sample.h

#ifndef FILTER_SAMPLE_H_ // Include guards
#define FILTER_SAMPLE_H_

#define ARM_MATH_CM7	// Use ARM Cortex M7
#define __FPU_PRESENT 1		// Does this device have a floating point unit?
#include <arm_math.h>	// Include CMSIS header
#include <cmsis_gcc.h>

// Link with library: libarm_cortexM4_mathL.a (or equivalent)
// Add CMSIS/Lib/GCC to the library search path
// Add CMSIS/Include to the include search path
extern float32_t filter_sample_coefficients[20];
static const int filter_sample_numStages = 4;

typedef struct
{
	arm_biquad_casd_df1_inst_f32 instance;
	float32_t state[16];
	float32_t output;
} filter_sampleType;


filter_sampleType *filter_sample_create( void );
void filter_sample_destroy( filter_sampleType *pObject );
void filter_sample_init( filter_sampleType * pThis );
void filter_sample_reset( filter_sampleType * pThis );
#define filter_sample_writeInput( pThis, input )  \
	arm_biquad_cascade_df1_f32( &pThis->instance, &input, &pThis->output, 1 );

#define filter_sample_readOutput( pThis )  \
	pThis->output

int filter_sample_filterBlock( filter_sampleType * pThis, float * pInput, float * pOutput, unsigned int count );


#define filter_sample_outputToFloat( output )  \
	(output)

#define filter_sample_inputFromFloat( input )  \
	(input)

#endif // FILTER_SAMPLE_H_
