/* ==============================================================================
System Name:  	HVACI_Sensorless

File Name:		HVACI_Sensorless.h

Description:	Primary system header file for the Real Implementation of Sensorless  
          		Field Orientation Control for Induction Motor

Originator:		Digital control systems Group - Texas Instruments

 
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2010	Version 1.0
=================================================================================  */

/*-------------------------------------------------------------------------------
Next, Include project specific include files.
-------------------------------------------------------------------------------*/
#include "aci_fe.h"        		// Include header for the ACIFE object
#include "aci_fe_const.h"   	// Include header for the ACIFE_CONST object
#include "aci_se.h"        		// Include header for the ACISE object
#include "aci_se_const.h"   	// Include header for the ACISE_CONST object
#include "adapt_cur_mod.h"		// Include header for the ADAPT_CUMOD object
#include "switches.h"			// Switch definitions
#include "static_curve.h"		// Include header for the STATIC CURVE object
#include "park.h"       		// Include header for the PARK object 
#include "ipark.h"       		// Include header for the IPARK object 
#include "pid_reg3.h"       	// Include header for the PIDREG3 object 
#include "clarke.h"         	// Include header for the CLARKE object 
#include "svgen_dq.h"       	// Include header for the SVGENDQ object 
#include "rampgen.h"        	// Include header for the RAMPGEN object 
#include "rmp_cntl.h"       	// Include header for the RMPCNTL object 
#include "volt_calc.h"      	// Include header for the PHASEVOLTAGE object 
#include "speed_fr.h"			// Include header for the SPEED_MEAS_QEP object
#include "inertia_emulator.h"	// Include header for the INERTIA_EMULATOR object
#include "static_curve_AD.h"	// Include header for the STATIC CURVE AD object
#include "wind_pattern.h"		// Include header for the WIND PATTERN object

#if (DSP2833x_DEVICE_H==1)
#include "f2833xpwm_AD.h"			// Include header for the PWMGEN object
#include "f2833xpwmdac.h"       // Include header for the PWMDAC object
#include "f2833xqep.h"        	// Include header for the QEP object
#include "f2833xileg_vdc_AD.h" 	// Include header for the ILEG2DCBUSMEAS object
#include "f2833xcap.h"        	// Include header for the CAP object
#endif

#include "dlog4ch-HVACI_Sensorless.h"			// Include header for the DLOG_4CH object

//===========================================================================
// No more.
//===========================================================================
