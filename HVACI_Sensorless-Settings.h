/* =================================================================================
File name:  HVACI_Sensorless-Settings.H                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Incremental Build Level control file.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 02-09-2010	Version 1.0
=================================================================================  */
#ifndef PROJ_SETTINGS_H

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------


#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define tipo_motor 30	// tipo_motor puede asumir dos valores, 2 o 30, lo cual indica la potencia del motor que se utiliza en HP
#define SQRT2 1.414213562
#define PI 3.14159265358979

// Define the system frequency (MHz)
#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP280x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 100
#elif (DSP2833x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 150
#endif


//Define system Math Type
// Select Floating Math Type for 2833x
// Select IQ Math Type for 2803x 
#if (DSP2803x_DEVICE_H==1)
#define MATH_TYPE 0 
#elif (DSP2833x_DEVICE_H==1)
#define MATH_TYPE 1
#endif



// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

#if (tipo_motor==30)
	// Define the electrical motor parametes (30HP Siemens en triángulo)
	#define RS 		0.1560		    // Stator resistance (ohm)		0.4680/3
	#define RR   	0.2039		    // Rotor resistance (ohm)		0.6116/3
	#define LS   	0.0381    	  	// Stator inductance (H)		0.1142/3
	#define LR   	0.0381	  		// Rotor inductance (H)			0.1142/3
	#define LM   	0.0364	   		// Magnatizing inductance (H)	0.1093/3
	#define POLES  	4				// Number of poles				4
	#define POLE_PAIRS  		2	// Number of pole pairs			2

	// Define the base quantites for PU system conversion
	#define BASE_VOLTAGE    660         // Base peak phase voltage (volt) con resistencias de 11Kohm
	#define BASE_CURRENT    90.3        // Base peak phase current (amp) para Honeywell (100HP con Rp=33.2ohm)
	#define BASE_FREQ      	120         // Base electrical frequency (Hz)
	#define BASE_TORQUE      8154.09	         // Base peak torque (N*m)
	#define BASE_WIND_SPEED    15        // Base peak speed (m/s)
	#define IsqFactor    9.5641        // Factor para la estimación de Isq mediante el torque
	#define TOR_NOM		142.4182		// Torque nominal de la máquina

#elif (tipo_motor==2)
	// Define the electrical motor parametes (2HP Siemens en estrella)
	#define RS 		6.042		    // Stator resistance (ohm)
	#define RR   	2.954		    // Rotor resistance (ohm)
	#define LS   	0.3159    	  	// Stator inductance (H)
	#define LR   	0.3159	  		// Rotor inductance (H)
	#define LM   	0.3005	   		// Magnatizing inductance (H)
	#define POLES  	4				// Number of poles
	#define POLE_PAIRS  		2	// Number of pole pairs

	// Define the base quantites for PU system conversion
	#define BASE_VOLTAGE    660         // Base peak phase voltage (volt) con resistencias de 11Kohm
	#define BASE_CURRENT    8.21        // Base peak phase current (amp) para Honeywell
	#define BASE_FREQ      	120         // Base electrical frequency (Hz)
	#define BASE_TORQUE      67.4041	         // Base peak torque (N*m)
	#define BASE_WIND_SPEED    15        // Base peak speed (m/s)
	#define IsqFactor    1.1661        // Factor para la estimación de Isq mediante el torque todo
	#define TOR_NOM		9.4945		// Torque nominal de la máquina todo
#endif

#endif
