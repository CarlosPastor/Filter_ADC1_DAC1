/*
 * filter_sample.c
 *
 *  Created on: 17 dic. 2016
 *      Author: alienBot
 */

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

#include <stdlib.h> // For malloc/free
#include <string.h> // For memset

#include "filter_sample.h"

float32_t filter_sample_coefficients[20] =
{
// Scaled for floating point

    0.45295337422636567, -0.9059067484527313, 0.45295337422636567, 1.9422872006860015, -0.9565065483590811,		// b0, b1, b2, a1, a2
    0.25, -0.5, 0.25, 1.9721443590268326, -0.9837984465718094,													// b0, b1, b2, a1, a2
    0.001953125, 0.00390625, 0.001953125, 1.9300834735701295, -0.9494140933241926,								// b0, b1, b2, a1, a2
    0.001953125, 0.00390625, 0.001953125, 1.952476317713594, -0.9767141915475573								// b0, b1, b2, a1, a2

};


filter_sampleType *filter_sample_create( void )
{
	filter_sampleType *result = (filter_sampleType *)malloc( sizeof( filter_sampleType ) );	// Allocate memory for the object
	filter_sample_init( result );											// Initialize it
	return result;																// Return the result
}

void filter_sample_destroy( filter_sampleType *pObject )
{
	free( pObject );
}

 void filter_sample_init( filter_sampleType * pThis )
{
	arm_biquad_cascade_df1_init_f32(&pThis->instance, filter_sample_numStages, filter_sample_coefficients, pThis->state );
	filter_sample_reset( pThis );
}

 void filter_sample_reset( filter_sampleType * pThis )
{
	memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
	pThis->output = 0;									// Reset output

}

 int filter_sample_filterBlock( filter_sampleType * pThis, float * pInput, float * pOutput, unsigned int count )
{
	arm_biquad_cascade_df1_f32(&pThis->instance, pInput, pOutput, count );
	return count;
}


