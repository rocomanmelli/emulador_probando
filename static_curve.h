/* =================================================================================
File name:       static_curve.h (IQ version)                    
                    
Originator:	Joaquin Ezpeleta

Description: 
Header file containing constants, data type, and macro definitions for the STATIC_CURVE.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 12-26-2013	Version 1.0                                                  
------------------------------------------------------------------------------*/
#ifndef __STATIC_CURVE_H__
#define __STATIC_CURVE_H__

#define STATIC_CURVE_DATA_POINTS 19

typedef struct {  _iq  WSpeed;				// Input: wind speed
				  _iq  TSpeed;				// Input: turbine speed
				  _iq  Torque;				// Output: turbine torque for given wind speed and turbine speed
				  _iq  Rho;  				// Constant: air density
				  _iq  Radius;	 			// Constant: turbine radius
				  _iq  Gain;				// Constant: gain
				  _iq  Offset;				// Constant: offset
				  	  	  	  	  	  	  	// Output = (Torque*Gain) + Offset
				  _iq  Staticcurvex[STATIC_CURVE_DATA_POINTS];	// Constant: x values of CT vs Lambda static curve					
				  _iq  Staticcurvey[STATIC_CURVE_DATA_POINTS];	// Constant: y values of CT vs Lambda static curve
				  _iq  CT;					// Variable: torque coefficient
				  _iq  Lambda;				// Variable: tip-speed ratio
		 	 	} STATIC_CURVE;	            

//typedef STATIC_CURVE *STATIC_CURVE_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the STATIC_CURVE object.
-----------------------------------------------------------------------------*/                     
#define STATIC_CURVE_DEFAULTS {_IQ(0),		\
                          _IQ(0),			\
                          _IQ(0),			\
                          _IQ(1), 			\
                          _IQ(2),			\
                          _IQ(0.25),			\
                          _IQ(0),			\
                          {_IQ(0),_IQ(0.5),_IQ(1),_IQ(1.25),_IQ(1.5),_IQ(2),_IQ(2.5),_IQ(2.75),_IQ(3),_IQ(3.25),_IQ(3.5),_IQ(3.75),_IQ(4),_IQ(4.25),_IQ(4.5),_IQ(5),_IQ(5.5),_IQ(6),_IQ(6.5)}, \
						  {_IQ(0.004),_IQ(0.004),_IQ(0.008),_IQ(0.01),_IQ(0.025),_IQ(0.054),_IQ(0.075),_IQ(0.078),_IQ(0.08),_IQ(0.082),_IQ(0.081),_IQ(0.077),_IQ(0.074),_IQ(0.066),_IQ(0.054),_IQ(0.028),_IQ(0),_IQ(-0.028),_IQ(-0.054)}, \
                          _IQ(0),			\
                          _IQ(0),			\
              			  }

/*------------------------------------------------------------------------------
	STATIC_CURVE Macro Definition
------------------------------------------------------------------------------*/

int i = 0;

#define STATIC_CURVE_MACRO(v)																						\
																													\
	v.Lambda = _IQdiv(_IQmpy(v.TSpeed,v.Radius),v.WSpeed);															\
	for(i=0; i<STATIC_CURVE_DATA_POINTS-1; i++)																		\
    {																												\
    	_iq diffx = 0;																								\
    	_iq diffn = 0;																								\
        if ( (v.Staticcurvex[i] <= v.Lambda) && (v.Staticcurvex[i+1] >= v.Lambda))									\
		{																											\
            diffx = v.Lambda - v.Staticcurvex[i];																	\
            diffn = v.Staticcurvex[i+1] - v.Staticcurvex[i];														\
            v.CT= v.Staticcurvey[i]+ _IQmpy((v.Staticcurvey[i+1]-v.Staticcurvey[i]),_IQdiv(diffx,diffn)); 			\
     		break;																									\
		}																											\
    }																												\
	v.Torque = v.Offset+_IQmpy(v.Gain,_IQmpy(_IQ(0.5),_IQmpy(v.Rho,_IQmpy(_IQ(PI),_IQmpy(v.Radius,_IQmpy(v.Radius,_IQmpy(v.Radius,_IQmpy(v.CT,_IQmpy(v.WSpeed,v.WSpeed)))))))));
#endif // __PARK_H__
