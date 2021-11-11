/* =================================================================================
File name: wind_pattern.h

Originator:	Gonzalo Asad

Created on: 4/7/2016

Description:
Header file containing constants, data type, and macro definitions for the WIND_PATTERN.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 4/7/2016	Version 1.0
------------------------------------------------------------------------------*/

#ifndef WIND_PATTERN_H_
#define WIND_PATTERN_H_


typedef struct {  float WSpeed;				// Input: wind speed
				  float Noise;				// Input: noise
				  float L;
				  float Ksigma;
				  float Kf;
				  float Tf;
				  float State;
				  float Rate;
				  float I0;
				  float	Viento;
				  _iq Out;
		 	 	} WIND_PATTERN;

//typedef WIND_PATTERN *WIND_PATTERN_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the WIND_PATTERN object.
-----------------------------------------------------------------------------*/
#define WIND_PATTERN_DEFAULTS {0.0,		\
		   	   	   	   	   	   0.0,		\
							   100.0,	\
							   0.25,	\
							   0.0,		\
							   0.0,		\
							   0.0,		\
							   0.0,		\
							   0.0,		\
							   0.0,		\
							   _IQ(0),  \
							}

/*------------------------------------------------------------------------------
	WIND_PATTERN Macro Definition
------------------------------------------------------------------------------*/
#define WIND_PATTERN_MACRO(v)									\
			if(v.WSpeed <= 0)									\
		 	 	{												\
		 	 		v.WSpeed=0.001;								\
		 	 	}												\
			v.Tf = v.L/v.WSpeed;								\
			v.Kf = sqrt((2.0*PI*v.Tf)/(4.2065*0.01));			\
			v.Rate = ((v.Kf*v.Noise)-v.State)/v.Tf;				\
			v.State = v.I0 + 0.01*v.Rate;						\
			v.I0=v.State;										\
			v.Viento=v.State*v.WSpeed*v.Ksigma+v.WSpeed;		\
			v.Out=_IQ(v.Viento/BASE_WIND_SPEED);
/*Se trabaja en flotante, a la salida se la divide por WIND_BASE para pasarla a PU y luego se la convierte a IQ (viento=_IQ(salida_PU))*/
#endif /* WIND_PATTERN_H_ */
