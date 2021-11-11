#ifndef ADAPT_CUR_MOD_H_
#define ADAPT_CUR_MOD_H_

typedef struct 	{ _iq  IDs; 		// Input: Syn. rotating d-axis current (pu)
				  _iq  IQs;			// Input: Syn. rotating q-axis current (pu)
			      _iq  Wr;			// Input: Rotor electrically angular velocity (pu)
				  _iq  IMDs;		// Variable: Syn. rotating d-axis magnetizing current (pu)
				  _iq  Theta;		// Output: Rotor flux angle (pu)
				  _iq  Torque;		// Output: Electromagnetic torque (pu)
			      _iq  Kr;			// Parameter: constant using in magnetizing current calculation
			      _iq  Kt;			// Parameter: constant using in slip calculation
			      _iq  K;			// Parameter: constant using in rotor flux angle calculation
			      _iq  Tr_cm;		// Parameter: rotor time constant
			      _iq  Ts;			// Parameter: sampling period (sec)
			      _iq  fb;			// Parameter: base electrical frequency (Hz)
				} CURMOD;

typedef CURMOD *CURMOD_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the CURMOD object.
-----------------------------------------------------------------------------*/
#define CURMOD_DEFAULTS { 0,0,0,0,0,0,0,0,0,0,0,0 }

/*------------------------------------------------------------------------------
 CUR_MOD Macro Definition
 //Verdadero par: multiplicar Torque por 20*20*2 (I base * I base * np)
------------------------------------------------------------------------------*/
	_iq Wslip, We;

#define CUR_MOD_MACRO(v)								\
	v.Kr = v.Ts/v.Tr_cm;								\
	v.Kt = 1/(v.Tr_cm*2*PI*v.fb);						\
	v.K = v.Ts*v.fb;									\
	v.IMDs +=  _IQmpy(v.Kr,(v.IDs - v.IMDs));			\
	v.Torque = (3/2)*(1-0.0951)*LS*_IQmpy(v.IMDs,v.IQs);\
	Wslip = _IQdiv(_IQmpy(v.Kt,v.IQs),v.IMDs);			\
	We = v.Wr + Wslip;									\
	v.Theta +=  _IQmpy(v.K,We);							\
														\
    if (v.Theta > _IQ(1))								\
       v.Theta -=  _IQ(1);								\
    else if (v.Theta < _IQ(0))							\
       v.Theta += _IQ(1);

#endif /* ADAPT_CUR_MOD_H_ */
