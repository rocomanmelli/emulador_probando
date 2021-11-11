/* =================================================================================
File name:       inertia_emulator.h (IQ version)

Originator:	Joaquin Ezpeleta

Description:
Header file containing constants, data type, and macro definitions for the INERTIA_EMULATOR.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 12-26-2013	Version 1.0
------------------------------------------------------------------------------*/
#ifndef __INERTIA_EMULATOR_H__
#define __INERTIA_EMULATOR_H__

typedef struct {  _iq  Speed;				// Input: shaft speed(pu)
				  _iq  SpeedF;				// Input: shaft filtered speed(pu)
				  _iq  TorqueRef;			// Input: initial torque reference(pu)
				  _iq  TorqueOut;			// Output: adjusted torque reference(pu)
				  _iq  TorqueEmul;			// Emulated torque(pu) todo
				  _iq  PrevSpeed;			// Variable: previous shaft speed
				  _iq  GearRatio;  			// Constant: gear ratio (Escrito de la forma 1/x)
				  _iq  bWT;	 				// Constant: turbine friction coefficient bWT=bwt*PUW/PUT
				  _iq  bM;					// Constant: motor friction coefficient bM=bm*PUW/PUT
				  _iq  JWT;					// Constant: turbine inertia JWT=Jwt*PUW/PUT
				  _iq  JM;					// Constant: motor inertia JM=Jm*PUW/PUT
		 	 	} INERTIA_EMULATOR;

//typedef INERTIA_EMULATOR *INERTIA_EMULATOR_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the INERTIA_EMULATOR object.
-----------------------------------------------------------------------------*/
#define INERTIA_EMULATOR_DEFAULTS {_IQ(0),	\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(0), 			\
                          _IQ(1),			\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(0),			\
              			  }

/*------------------------------------------------------------------------------
	INERTIA_EMULATOR Macro Definition
------------------------------------------------------------------------------*/

#define INERTIA_EMULATOR_MACRO(v)																					\
	if(v.SpeedF > 1 )																								\
		 	 	{v.SpeedF=v.PrevSpeed;}																				\
	v.TorqueEmul = -_IQmpy((v.SpeedF-v.PrevSpeed)/T,_IQmpy(v.JWT,_IQmpy(v.GearRatio,v.GearRatio))-v.JM);\
	v.TorqueOut = _IQmpy(v.TorqueRef,v.GearRatio) + v.TorqueEmul -_IQmpy(v.Speed,_IQmpy(v.bWT,_IQmpy(v.GearRatio,v.GearRatio))-v.bM);													\
	v.PrevSpeed = v.SpeedF;																							\
	if (v.TorqueOut <= -1)																							\
		 v.TorqueOut = -1;																							\
	else if	(v.TorqueOut >= 1)																						\
		 v.TorqueOut = 1;
#endif // __INERTIA_EMULATOR_H__
