/* =================================================================================
File name: static_curve_AD.h

Originator:	Gonzalo Asad

Created on: 1/7/2016

Description:
Header file containing constants, data type, and macro definitions for the STATIC_CURVE_AD.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 1/7/2016	Version 1.0
------------------------------------------------------------------------------*/

#ifndef __STATIC_CURVE_AD_H__
#define __STATIC_CURVE_AD_H__

typedef struct {  _iq  WSpeed;				// Input: wind speed
				  _iq  TSpeed;				// Input: turbine speed
				  _iq  Torque;				// Output: turbine torque for given wind speed and turbine speed
				  _iq  Gain;				// Constant: gain
				  _iq  Offset;				// Constant: offset
				  	  	  	  	  	  	  	// Output = (Torque*Gain) + Offset
				  _iq  CP;					// Variable: power coefficient
				  _iq  Lambda;				// Variable: tip-speed ratio
				  _iq  PitchAngle;			// Input: blade pitch angle (deg)
				  _iq  Lambda_i;
				  _iq  CP_nom;
				  _iq  Lambda_nom;
				  _iq  c1;
				  _iq  c2;
				  _iq  c3;
				  _iq  c4;
				  _iq  c5;
				  _iq  c6;
				  _iq  WSpeed_aux;
				  _iq  TSpeed_aux;
				  _iq  CP_pu;
				  _iq  Lambda_pu;
		 	 	} STATIC_CURVE_AD;

//typedef STATIC_CURVE_AD *STATIC_CURVE_AD_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the STATIC_CURVE_AD object.
-----------------------------------------------------------------------------*/
#define STATIC_CURVE_AD_DEFAULTS {_IQ(0),	\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(1),			\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(0),			\
						  _IQ(0),			\
						  _IQ(0),			\
						  _IQ(0.48),		\
						  _IQ(8.1),			\
                          _IQ(0.5176),		\
                          _IQ(116),			\
                          _IQ(0.4),			\
                          _IQ(5),			\
						  _IQ(21),			\
						  _IQ(0.0068),		\
						  _IQ(0),			\
						  _IQ(0),			\
						  _IQ(0),			\
						  _IQ(0),			\
						}

/*------------------------------------------------------------------------------
	STATIC_CURVE_AD Macro Definition
------------------------------------------------------------------------------*/

#define STATIC_CURVE_AD_MACRO(v)																															\
																																							\
	if (v.WSpeed <= 0)																																		\
		 v.WSpeed_aux = 0.001;																																\
	else																																					\
		 v.WSpeed_aux = v.WSpeed;																															\
		 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	\
	if (v.TSpeed <= 0)																																		\
		 v.TSpeed_aux = 0.001;																																\
	else																																					\
		 v.TSpeed_aux = v.TSpeed;																															\
		 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  	 	 	 	 	 	  	 	\
	v.Lambda_pu = _IQdiv(v.TSpeed,v.WSpeed_aux);																											\
	v.Lambda = _IQmpy(v.Lambda_pu,v.Lambda_nom);																											\
																																							\
	if (v.Lambda <= 0)																																		\
		 v.Lambda = 0.001;																																	\
		 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	\
	v.Lambda_i = _IQdiv(1,_IQdiv(1,v.Lambda+_IQmpy(0.08,v.PitchAngle))-_IQdiv(0.035,_IQmpy(v.PitchAngle,_IQmpy(v.PitchAngle,v.PitchAngle))+1));				\
	v.CP = _IQmpy(v.c1,_IQmpy(_IQdiv(v.c2,v.Lambda_i)-_IQmpy(v.c3,v.PitchAngle)-v.c4,_IQexp(_IQmpy(-1,_IQdiv(v.c5,v.Lambda_i)))))+_IQmpy(v.c6,v.Lambda);	\
	v.CP_pu = _IQdiv(v.CP,v.CP_nom);																														\
	v.Torque = v.Offset+_IQdiv(_IQmpy(v.Gain,_IQmpy(v.CP_pu,_IQmpy(v.WSpeed,_IQmpy(v.WSpeed,v.WSpeed)))),v.TSpeed_aux);
#endif /* __STATIC_CURVE_AD_H__ */
