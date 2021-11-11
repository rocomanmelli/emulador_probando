/* =================================================================================
File name:       PRBS.h (IQ version)

Originator:	Joaquin Ezpeleta

Description:
Header file containing constants, data type, and macro definitions for the PRBS.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 12-26-2013	Version 1.0
------------------------------------------------------------------------------*/
#ifndef __PRBS_H__
#define __PRBS_H__

typedef struct {  unsigned int Prescaler;	// Parameter: period prescaler
				  unsigned int size;		// Parameter: size
				  _iq Gain;					// Parameter: gain
				  unsigned int Counter;		// Variable: internal counter
				  unsigned int m;			// Variable: m
				  unsigned int n;			// Variable: n
				  unsigned int shift_reg;	// Variable: shift register
				  _iq Out;					// Output: scaled PRBS
		 	 	} PRBS;

//typedef PRBS *PRBS_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PRBS object.
-----------------------------------------------------------------------------*/
#define PRBS_DEFAULTS {	10,		\
        				16,		\
        				_IQ(0.05),	\
        				0,		\
        				0,		\
        				0,		\
        				0xFFFF,	\
        				_IQ(0)	\
              		}

/*------------------------------------------------------------------------------
	PRBS Macro Definition
------------------------------------------------------------------------------*/

const char tap[16] = {0, 0, 0, 2, 3, 3, 5, 6, 0, 5, 7, 9, 0, 0, 0, 14};

#define PRBS_MACRO(v)																					\
																										\
	if(v.Prescaler==v.Counter++){																		\
		v.m = (v.shift_reg >> (v.size - 1)) & 0x1;														\
		v.n = (v.shift_reg >> (tap[v.size] - 1)) & 0x1;													\
		v.shift_reg = (v.shift_reg << 1);																\
		if (v.m^v.n)																					\
			v.shift_reg |= 0x1;																			\
		v.Counter=0;																					\
	}																									\
	v.Out = v.Gain*v.m;

#endif // __PRBS_H__
