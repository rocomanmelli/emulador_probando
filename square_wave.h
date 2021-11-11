/* =================================================================================
File name:       square_wave.h (IQ version)

Originator:	Joaquin Ezpeleta

Description:
Header file containing constants, data type, and macro definitions for the INERTIA_EMULATOR.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 12-26-2013	Version 1.0
------------------------------------------------------------------------------*/
#ifndef __SQUARE_WAVE_H__
#define __SQUARE_WAVE_H__

typedef struct {  _iq  Period;				// Parameter: shaft speed
				  _iq  Value1;				// Input: initial torque reference
				  _iq  Value2;				// Output: adjusted torque reference
		 	 	} SQUARE_WAVE;

//typedef SQUARE_WAVE *SQUARE_WAVE_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SQUARE_WAVE object.
-----------------------------------------------------------------------------*/
#define SQUARE_WAVE {_IQ(0),	\
                          _IQ(0),			\
                          _IQ(0),			\
              			  }

/*------------------------------------------------------------------------------
	SQUARE_WAVE Macro Definition
------------------------------------------------------------------------------*/

#define SQUARE_WAVE_MACRO(v)																					\
																													\
	v.PrevSpeed = v.Speed;																							\

#endif // __SQUARE_WAVE_H__
