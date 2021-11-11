/*
 * EmuladorAD.c
 *
 *  Created on: 13/6/2016
 *      Authors: Asad & Dórdolo
 */


// Include header files used in the main function

#include "PeripheralHeaderIncludes.h"
#include "HVACI_Sensorless-Settings.h"
#include "IQmathLib.h"
#include "HVACI_Sensorless.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// Prototype statements for functions found within this file.
interrupt void MainISR(void);
interrupt void DesatISR(void);
void DeviceInit();
void MemCopy();
void InitFlash();
void delay_loop(long);
void TZ_Protection(void);
void TZ_Clear(void);
float AWGN_generator(void);
_iq FieldWeakening(_iq);


// Global variables used in this system
/*
volatile _iq vector[200]; //para datalogger simple
volatile int indice= 0; //para datalogger simple
*/

_iq VdTesting = _IQ(0);			// Vd reference (pu)
_iq VqTesting = _IQ(0);			// Vq reference (pu)
_iq IdRef = _IQ(0);				// Id reference (pu) (2.2 A)
_iq IqRef = _IQ(0);					// Iq reference (pu)
_iq SpeedRef = _IQ(0);			// Speed reference (pu)
_iq TorqueRef = _IQ(0);			// Referencia de torque para ensayo de inercia
_iq WindSpeed = _IQ(0);
_iq TurbineSpeed = _IQ(0);
_iq GearRatio = _IQ(1);			// Como se trabaja en PU debe ser 1
_iq Isdf    = _IQ(0);				// Corriente directa filtrada
_iq Isqf    = _IQ(0);				// Corriente en cuadratura filtrada
_iq Ieff    = _IQ(0);				// Corriente eficaz
_iq Vdcf    = _IQ(0);				// Tensión eficaz
_iq Speedf  = _IQ(0);				// Velocidad filtrada
_iq Speed_1  = _IQ(0);				// Velocidad (z^-1)
_iq TorqueHat = _IQ(0);				// Torque estimado según García
_iq TorqueHatGarciamod = _IQ(0);		// Torque estimado según paper de García con modificaciones
_iq TorRef = _IQ(0);
_iq IaOffset = _IQ(0);
_iq IbOffset = _IQ(0);
_iq modulo= _IQ(0);
_iq Vd=_IQ(0);
_iq Vq=_IQ(0);
_iq angulo=_IQ(0); // angulo electrico invertido

/*
 * vectores para implementar debilitamiento de campo a
 * frecuencias entre 50Hz y 60Hz
 */
const _iq speeds[] = { 0.4166, 0.42, 0.43, 0.44, 0.45, 0.46, 0.47, 0.48, 0.49, 0.5 };
const _iq currents[] = {0.2725, 0.2670, 0.2530, 0.2420, 0.2325, 0.2240, 0.2169, 0.2105, 0.2045, 0.1995};
Uint16 enable_weakening = 0;

volatile int CLEAR_FAULT=0;
int TRIP=0;

#if(tipo_motor==30)
	_iq Jwt    = _IQ(3.75);				// Inercia de la turbina
	_iq Jm    = _IQ(0.12);				// Inercia del motor
	_iq bwt    = _IQ(0.02);				// Rozamiento de la turbina
	_iq bm    = _IQ(0.005);				// Rozamiento del motor
#elif(tipo_motor==2)
	_iq Jwt    = _IQ(0.18);				// Inercia de la turbina
	_iq Jm    = _IQ(0.01312);			// Inercia del motor
	_iq bwt    = _IQ(0.01);				// Rozamiento de la turbina
	_iq bm    = _IQ(0.001508);			// Rozamiento del motor
#endif


//Se usan para eliminar el error del término integral, para evitar que haya saltos.
Uint16 STOP = 0;
Uint16 idBlocker=0;
Uint16 iqBlocker=0;
Uint16 spdBlocker=0;

float32 Temperature;	//Medición de temperatura

float32 T = 1.0/6000.0;    //0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h trabajando a 6Khz

Uint32 IsrTicker = 0;
Uint16 BackTicker = 0;

Uint16 ModoAD = 0;	//Elige el modo
Uint16 ChangeState = 15;

volatile Uint16 EnableFlag = FALSE;
volatile Uint16 iParkDsSwitch = SW_VD_TESTING;
volatile Uint16 iParkQsSwitch = SW_VQ_TESTING;
volatile Uint16 pidIqRefSwitch = SW_IQ_REF;
volatile Uint16 iParkAngleSwitch = SW_RG;
volatile Uint16 staticCurveTSpeedSwitch = SW_CONST_T_SPD;
volatile Uint16 staticCurveWSpeedSwitch = SW_CONST_W_SPD;

//Defines para Wind Shear y Tower Shadow (switch mentiroso, implementado con un 'if')
float32 U3 = 1;
int32 Kmax = 0;
Uint16 SW_WS_TS = 0;
Uint16 SW_ENS_INEM = 0;
Uint32 k = 0;

void delay_loop(long end)
{
	long i;
	for (i = 0; i < end; i++)
	{
		asm(" NOP");
		EALLOW;
		SysCtrlRegs.WDKEY = 0x55;
		SysCtrlRegs.WDKEY = 0xAA;
		EDIS;
	}
}

int16 VDC_FLAG=-1;
int16 SPEED_FLAG=-1;
int16 CLARKE_FLAG=-1;
int16 IEFF_FLAG=-1;
int16 OCP_FLAG=-1;
int16 DESAT_FLAG=-1;
int16 OVERTEMP_FLAG=-1;
int16 IQSWITCH_FLAG=-1;
int16 FLAG_AC_FAIL=-1;


Uint16 SpeedLoopPrescaler = 10;      // Speed loop prescaler
Uint16 SpeedLoopCount = 1;           // Speed loop counter
Uint16 Contador_AC_FAIL = 0;		 // Contador para AC-Line
Uint16 Contador_Ejecuciones = 1;	 // Contador para hacer las ejecuciones de macros

// Instance WIND PATERN
WIND_PATTERN patron_viento = WIND_PATTERN_DEFAULTS;

// Instance a static curve object
STATIC_CURVE_AD static_curve1 = STATIC_CURVE_AD_DEFAULTS;

// Instance an adaptive current model object
CURMOD cm1 = CURMOD_DEFAULTS;

// Instance a QEP interface driver
QEP qep1 = QEP_DEFAULTS;

// Instance a few transform objects (ICLARKE is added in SVGEN module)
CLARKE clarke1 = CLARKE_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

// Instance PID regulators to regulate the d and q  axis currents, and speed
PIDREG3 pid1_id = PIDREG3_DEFAULTS;
PIDREG3 pid1_iq = PIDREG3_DEFAULTS;
PIDREG3 pid1_spd = PIDREG3_DEFAULTS;

// Instance a PWM driver instance
PWMGEN pwm1 = PWMGEN_DEFAULTS;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs
SVGENDQ svgen_dq1 = SVGENDQ_DEFAULTS;

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;

// Instance a ramp controller to smoothly ramp the torque
RMPCNTL rc2 = RMPCNTL_DEFAULTS;

// Instance a ramp controller to smoothly ramp the Iq after the inertia emulator (runs only once per switching)
RMPCNTL rc3 = RMPCNTL_DEFAULTS;

// Instance a ramp controller to smoothly ramp the wind speed reference
RMPCNTL rc4 = RMPCNTL_DEFAULTS;

// Instance a ramp controller to smoothly ramp the turbine speed reference
RMPCNTL rc5 = RMPCNTL_DEFAULTS;

// Instance a ramp controller to smoothly ramp the flux
RMPCNTL rc6 = RMPCNTL_DEFAULTS;

//	Instance a ramp(sawtooth) generator to simulate an Angle
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//	Instance a phase voltage calculation
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;

// Instance a speed calculator based on QEP
SPEED_MEAS_QEP speed1 = SPEED_MEAS_QEP_DEFAULTS;

// Create an instance of INERTIA_EMULATOR Module
INERTIA_EMULATOR inertia_emulator1 = INERTIA_EMULATOR_DEFAULTS;	//GR (gear ratio) debería ser 8.3 (8)

#define BUFFER_LENGTH 230
#define PRE_BUFFER_LENGTH 20
#if !(PRE_BUFFER_LENGTH <= BUFFER_LENGTH)
	#error PRE_BUFFER_LENGTH must be <= BUFFER_LENGTH
#endif
volatile _iq speedBuffer[PRE_BUFFER_LENGTH+BUFFER_LENGTH];
volatile _iq speedRefBuffer[PRE_BUFFER_LENGTH+BUFFER_LENGTH];
volatile _iq iqBuffer[PRE_BUFFER_LENGTH+BUFFER_LENGTH];
volatile _iq idBuffer[PRE_BUFFER_LENGTH+BUFFER_LENGTH];
volatile _iq preSpeedBuffer[PRE_BUFFER_LENGTH];
volatile _iq preSpeedRefBuffer[PRE_BUFFER_LENGTH];
volatile _iq preIqBuffer[PRE_BUFFER_LENGTH];
volatile _iq preIdBuffer[PRE_BUFFER_LENGTH];
volatile int iBufferFilter = 0;
int preStepBufferIndex = 0;
int stepBufferIndex = PRE_BUFFER_LENGTH;
int currentPreBufferIndex = 0;
volatile int stepGo = 0;
volatile int stepRecord = 0;
volatile int triggerArmed = 1;
volatile int stepSwitch = 2;
int stepCounter = 0;
volatile int stepPrescaler = 200;
volatile _iq stepValue = _IQ(0);
volatile _iq* triggerPointer = &speed1.Speed;
volatile _iq triggerValue = _IQ(0.3);

void main(void)

 {
	DINT;
	DeviceInit(); // Device Life support & GPIO
	//InitSysCtrl();
// Only used if running from FLASH
// Note that the variable FLASH is defined by the compiler

#ifdef FLASH
// Copy time critical code and Flash setup code to RAM
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the linker files.
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
#endif //(FLASH)

//Enable Buffers PWM
	GpioDataRegs.GPACLEAR.bit.GPIO15=1;

// Espera de AC-Line y respuesta con /Charge
   GpioDataRegs.GPASET.bit.GPIO11=1;

//Charge está invertido, por lo que ya se recibió la señal en alto de AC_Line al energizar la máquina
//Mediante el SET del bit de Charge, bajamos la señal y AC_Line se mantendrá en alto

   while(1)
   {
	   if(GpioDataRegs.GPADAT.bit.GPIO13 == 1) //Si AC_Line está en bajo (recordar que está negado)
	   {
		   //Si en 20 segundos no llega la señal de AC_Line, levanta una bandera
	   	   delay_loop(100000); //10ms
	   	   Contador_AC_FAIL=Contador_AC_FAIL+1;
	   	   if(Contador_AC_FAIL==2000)
	   	   {
	   	   	   FLAG_AC_FAIL=1;
	   	   	   while(1){}
	   	   }
	   }
	   else
	   {
		   break;
	   }
   }


// Waiting for enable flag set
   while (EnableFlag==FALSE)
    {
	   BackTicker++;
    }


// Initialize INERTIA EMULATOR module
   inertia_emulator1.JWT = _IQmpy(Jwt,BASE_FREQ*2*PI/POLE_PAIRS/BASE_TORQUE); //
   inertia_emulator1.JM = _IQmpy(Jm,BASE_FREQ*2*PI/POLE_PAIRS/BASE_TORQUE);
   inertia_emulator1.bWT = _IQmpy(bwt,BASE_FREQ*2*PI/POLE_PAIRS/BASE_TORQUE);
   inertia_emulator1.bM = _IQmpy(bm,BASE_FREQ*2*PI/POLE_PAIRS/BASE_TORQUE);
   inertia_emulator1.GearRatio = GearRatio;

// Initialize PWM module
    pwm1.PeriodMax = SYSTEM_FREQUENCY*1000000*T/2;  // Prescaler X1 (T1), ISR period = T x 1   deberia ser 12500 para 6KHz
    PWM_INIT_MACRO(pwm1)

// Initialize ADC module (F28xxILEG_VDC.H)
	ADC_MACRO_INIT()

// Initialize QEP module
    qep1.LineEncoder = 1000;
    qep1.MechScaler = _IQ30(0.25/qep1.LineEncoder);
    qep1.PolePairs = POLES/2;
    qep1.CalibratedAngle = 0;
	QEP_INIT_MACRO(qep1)

// Initialize the ADAPT_CUR_MOD constant module
	//cm1.Tr_cm = _IQ(LR/RR);
    cm1.Tr_cm = _IQ(0.3218); // ajuste de cte de tiempo rotórica para mejorar respuesta del sistema bajo carga.
	cm1.fb = _IQ(BASE_FREQ);
	cm1.Ts = _IQ(T);

// Initialize the rc1 module // Rampa de velocidad de ref
	rc1.RampDelayMax=10;

// Initialize the rc2 module // Rampa de torque
	rc2.RampDelayMax=1;

// Initialize the rc3 module // Rampa de Iq
	rc3.RampDelayMax=1;

// Initialize the rc4 module // Rampa de referencia de velocidad de viento
	rc4.RampDelayMax=1;

// Initialize the rc5 module // Rampa de referencia de velocidad de turbina
	rc5.RampDelayMax=1;

// Initialize the rc6 module // Rampa de Id
	rc6.RampDelayMax=1;

// Initialize the Speed module for QEP based speed calculation
    speed1.K1 = _IQ21(1/(BASE_FREQ*T));
    speed1.K2 = _IQ(1/(1+T*2*PI*5));  // Low-pass cut-off frequency
    speed1.K3 = _IQ(1)-speed1.K2;
    speed1.BaseRpm = 120*(BASE_FREQ/POLES);

// Initialize the RAMPGEN module
    rg1.StepAngleMax = _IQ(BASE_FREQ*T);

  if(tipo_motor==30)
  {
		// Initialize the PID_REG3 module for Id
			pid1_id.Kp = _IQ(0.876);				// Con los valores de MatLab Kp = Pi*(I0/V0); le dimos 10 veces mas
			pid1_id.Ki = _IQ(0.06);			// Con los valores de MatLab Ki = (Pi/Ii)*(I0/V0); le dimos x veces menos
			pid1_id.Kd = _IQ(0/T);
			pid1_id.Kc = _IQ(0.2);
			pid1_id.OutMax = _IQ(0.999);
			pid1_id.OutMin = _IQ(-0.999);

		// Initialize the PID_REG3 module for Iq
			pid1_iq.Kp = _IQ(0.876);
			pid1_iq.Ki = _IQ(0.06);
			pid1_iq.Kd = _IQ(0/T);
			pid1_iq.Kc = _IQ(0.2);
			pid1_iq.OutMax = _IQ(0.999);
			pid1_iq.OutMin = _IQ(-0.999);

		// Initialize the PID_REG3 module for speed
			pid1_spd.Kp = _IQ(0.75);				// valores ajustados empiricamente (reducido a la mitad)
			pid1_spd.Ki = _IQ(0.12);			// valores ajustados empiricamente
			pid1_spd.Kd = _IQ(0/(T*SpeedLoopPrescaler));
			pid1_spd.Kc = _IQ(0.2);
			pid1_spd.OutMax = _IQ(0.85);
			pid1_spd.OutMin = _IQ(-0.85);
  }
  else if(tipo_motor==2)
  {
	    // Initialize the PID_REG3 module for Id
			pid1_id.Kp = _IQ(0.9);				// Con los valores de MatLab Kp = Pi*(I0/V0)
			pid1_id.Ki = _IQ(T*400.0/0.9);		// Con los valores de MatLab Ki = (Pi/Ii)*(I0/V0)
			pid1_id.Kd = _IQ(0/T);
			pid1_id.Kc = _IQ(0.2);
			pid1_id.OutMax = _IQ(0.9);
			pid1_id.OutMin = _IQ(-0.9);

		// Initialize the PID_REG3 module for Iq
			pid1_iq.Kp = _IQ(0.9);				// Con los valores de MatLab Kp = Pi*(I0/V0)
			pid1_iq.Ki = _IQ(T*400.0/0.9);		// Con los valores de MatLab Ki = (Pi/Ii)*(I0/V0)
			pid1_iq.Kd = _IQ(0/T);
			pid1_iq.Kc = _IQ(0.2);
			pid1_iq.OutMax = _IQ(0.95);
			pid1_iq.OutMin = _IQ(-0.95);

		// Initialize the PID_REG3 module for speed
			pid1_spd.Kp = _IQ(20);				// Con los valores de MatLab Kp = Pw*(W0/(I0^2))
			pid1_spd.Ki = _IQ(T*20);			// Con los valores de MatLab Ki = (Pw/Iw)*(W0/(I0^2))
			pid1_spd.Kd = _IQ(0/(T*SpeedLoopPrescaler));
			pid1_spd.Kc = _IQ(0.2);
			pid1_spd.OutMax = _IQ(0.8);
			pid1_spd.OutMin = _IQ(-0.8);
  }
  else
  {
	  while(1){}
  }

// Enable NMI for DESAT
    XIntruptRegs.XNMICR.bit.ENABLE=1;
    XIntruptRegs.XNMICR.bit.SELECT=1;
    XIntruptRegs.XNMICR.bit.POLARITY=0x2;

// Reassign ISRs.
	EALLOW;	// This is needed to write to EALLOW protected registers
	PieVectTable.EPWM1_INT = &MainISR;
	//PieVectTable.XNMI= &DesatISR; // todo habilitar protección
	EDIS;

// Enable PIE group 3 interrupt 1 for EPWM1_INT
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Enable CNT_zero interrupt using EPWM1 Time-base
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // Enable EPWM1INT generation
    //EPwm1Regs.ETSEL.bit.INTSEL = 1;  // Enable interrupt CNT_zero event
    EPwm1Regs.ETSEL.bit.INTSEL = 2;  // Enable interrupt period event
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // Generate interrupt on the 1st event
	EPwm1Regs.ETCLR.bit.INT = 1;     // Enable more interrupts


// Configura interrupciones de TZ
	EALLOW;
	//Disparo por hardware en TZ1
		EPwm1Regs.TZSEL.bit.OSHT1 = 0;
		EPwm2Regs.TZSEL.bit.OSHT1 = 0;
		EPwm3Regs.TZSEL.bit.OSHT1 = 0;
	//Comportamiento de los IGBTs cuando se acciona la proteccion
	//EPWM1A, EPWM1B en bajo
		EPwm1Regs.TZCTL.bit.TZA = 0x2;
		EPwm1Regs.TZCTL.bit.TZB = 0x2;
	//EPWM2A, EPWM2B en bajo
		EPwm2Regs.TZCTL.bit.TZA = 0x2;
		EPwm2Regs.TZCTL.bit.TZB = 0x2;
	//EPWM3A, EPWM3B en bajo
		EPwm3Regs.TZCTL.bit.TZA = 0x2;
		EPwm3Regs.TZCTL.bit.TZB = 0x2;
	EDIS;

// Enable CPU INT3 for EPWM1_INT:
	IER |= M_INT3;

// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

	while(1)
	{
		//La siguiente protección fue desactivada porque está implementada abajo
		//Quisimos ponerla acá también para responder más rápidamente, pero con el ruido que tenemos saltaba constantemente
		/*if(GpioDataRegs.GPADAT.bit.GPIO14 == 1)
		{
			if(!STOP)
			{
			DESAT_FLAG=GpioDataRegs.GPADAT.bit.GPIO14==1?1:0;
			}
			STOP=2;
			TZ_Protection();
		}
		*/
//---------------------------------------funcion para levantar fallas hasta el modo 3------------------------------------------
		if (CLEAR_FAULT==1)
		{

			if(ModoAD==0){
				VdTesting=0;
				VqTesting=0;
				SpeedRef=0;
			}

			if(ModoAD==1){
				IdRef=0;
				IqRef=0;
				SpeedRef=0;
			}

			if(ModoAD==2){
				IdRef=0;
				IqRef=0;
			}

			if(ModoAD==3){
			IdRef=0;
			SpeedRef=0;
			}

		STOP=0;
		VDC_FLAG=-1;
		SPEED_FLAG=-1;
		IEFF_FLAG=-1;
		CLARKE_FLAG=-1;
		OCP_FLAG=-1;
		OVERTEMP_FLAG=-1;
		DESAT_FLAG=-1;

		TZ_Clear();
		CLEAR_FAULT=0;

		}
//---------------------------------------------------------------------------------------------------------------------------------
		if (ModoAD<=8)
			{
				if(ChangeState != ModoAD)  //Si cambio de modo, la velocidad se mantiene en el mismo valor.
				{	
					ChangeState=ModoAD;
					SpeedRef = speed1.Speed;
					rc1.SetpointValue = speed1.Speed;
				}	
				switch(ModoAD)
				{
					case 0:												//Probamos con valores de referencia para verificar que se disparen los PWM correctamente
						iParkDsSwitch = SW_VD_TESTING;
						iParkQsSwitch = SW_VQ_TESTING;
						pidIqRefSwitch = SW_IQ_REF;
						iParkAngleSwitch = SW_RG;
						staticCurveTSpeedSwitch = SW_CONST_T_SPD;
						staticCurveWSpeedSwitch = SW_CONST_W_SPD;
						SW_WS_TS = 0;
					break;
					case 1:												//Agregamos los PID de corriente con SP de corriente constante
						iParkDsSwitch = SW_PID_ID;						
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_IQ_REF;
						iParkAngleSwitch = SW_RG;
						staticCurveTSpeedSwitch = SW_CONST_T_SPD;
						staticCurveWSpeedSwitch = SW_CONST_W_SPD;
						SW_WS_TS = 0;
					break;
					case 2:												//Agregamos la medición del ángulo por CURMOD
						iParkDsSwitch = SW_PID_ID;
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_IQ_REF;
						iParkAngleSwitch = SW_CM;
						staticCurveTSpeedSwitch = SW_CONST_T_SPD;
						staticCurveWSpeedSwitch = SW_CONST_W_SPD;
						SW_WS_TS = 0;
					break;
					case 3:												//Cambiamos SP de Iq constante por PID de velocidad
						iParkDsSwitch = SW_PID_ID;
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_PID_SPD;
						iParkAngleSwitch = SW_CM;
						staticCurveTSpeedSwitch = SW_CONST_T_SPD;
						staticCurveWSpeedSwitch = SW_CONST_W_SPD;
						SW_WS_TS = 0;
					break;
					case 4:												//Cambiamos SP de Iq por curvas estáticas (Asad-Dórdolo: P.U.) con SP de vel de turbina constante + rampa de torque
						iParkDsSwitch = SW_PID_ID;
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_TORQUE_RAMP;
						iParkAngleSwitch = SW_CM;
						staticCurveTSpeedSwitch = SW_CONST_T_SPD;
						staticCurveWSpeedSwitch = SW_CONST_W_SPD;
						SW_WS_TS = 0;
					break;
					case 5:												//Cambiamos SP de Iq por curvas estáticass (Asad-Dórdolo: P.U.) con SP de vel de turbina medido + rampa de torque
						iParkDsSwitch = SW_PID_ID;
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_TORQUE_RAMP;
						iParkAngleSwitch = SW_CM;
						staticCurveTSpeedSwitch = SW_MEAS_T_SPD;
						staticCurveWSpeedSwitch = SW_CONST_W_SPD;
						//SW_WS_TS = 0;
					break;
					case 6:												//Cambiamos SP de Iq por curvas estáticas (Asad-Dórdolo: P.U.) con SP de vel de turbina medido + emulador de inercia
						iParkDsSwitch = SW_PID_ID;
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_INERTIA_EMULATOR;
						iParkAngleSwitch = SW_CM;
						staticCurveTSpeedSwitch = SW_MEAS_T_SPD;
						staticCurveWSpeedSwitch = SW_CONST_W_SPD;
						//SW_WS_TS = 0;
					break;
					case 7:												//Agregamos patrón de viento al caso anterior
						iParkDsSwitch = SW_PID_ID;
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_INERTIA_EMULATOR;
						iParkAngleSwitch = SW_CM;
						staticCurveTSpeedSwitch = SW_MEAS_T_SPD;
						staticCurveWSpeedSwitch = SW_PATTERN_W_SPD;
						//SW_WS_TS = 0;
					break;
					case 8:												//Agregamos Wind Shear y Tower Shadow
						iParkDsSwitch = SW_PID_ID;
						iParkQsSwitch = SW_PID_IQ;
						pidIqRefSwitch = SW_INERTIA_EMULATOR;
						iParkAngleSwitch = SW_CM;
						staticCurveTSpeedSwitch = SW_MEAS_T_SPD;
						staticCurveWSpeedSwitch = SW_PATTERN_W_SPD;
						SW_WS_TS = 1;
					break;
				}
			}

		// Para implementar protección
		if(ModoAD > 3)
		{
			if(speed1.Speed >= 0)
			{
				SpeedRef = 0.416;
			}
			else
			{
				SpeedRef = 0;
			}
		}

	}
} //END MAIN CODE




//MainISR
interrupt void MainISR(void)
{

// Verifying the ISR
    IsrTicker++;
/*
    if(IdRef>0.04){
    	if(indice<200){
    		vector[indice]=park1.Ds;
    		indice++;
    	}
    }
*/

     if(triggerArmed && (*triggerPointer > triggerValue)){
    	stepRecord = 1;
    	triggerArmed = 0;
    }

    if(stepGo){
    	switch(stepSwitch){
    	case 0:
    		SpeedRef = stepValue;
    		break;
    	case 1:
    		IqRef = stepValue;
    		break;
    	case 2:
    		IdRef = stepValue;
    		break;
    	}
    	stepRecord=1;
    	stepGo=0;
    }

    if(stepCounter < stepPrescaler){
    	stepCounter++;
    }else{
    	stepCounter = 0;
    	if(stepRecord){
    		if(stepBufferIndex < 2*PRE_BUFFER_LENGTH)
    		{
    			currentPreBufferIndex = (preStepBufferIndex+stepBufferIndex-PRE_BUFFER_LENGTH)%PRE_BUFFER_LENGTH;
        		speedBuffer[stepBufferIndex-PRE_BUFFER_LENGTH] = preSpeedBuffer[currentPreBufferIndex];
        		speedRefBuffer[stepBufferIndex-PRE_BUFFER_LENGTH] = preSpeedRefBuffer[currentPreBufferIndex];
        		iqBuffer[stepBufferIndex-PRE_BUFFER_LENGTH] = preIqBuffer[currentPreBufferIndex];
        		idBuffer[stepBufferIndex-PRE_BUFFER_LENGTH] = preIdBuffer[currentPreBufferIndex];
    		}
    		speedBuffer[stepBufferIndex] = speed1.Speed;
    		speedRefBuffer[stepBufferIndex] = pid1_spd.Ref;
    		//iqBuffer[stepBufferIndex] = (iBufferFilter?Isqf:park1.Qs);
    		iqBuffer[stepBufferIndex] = AdcMirror.ADCRESULT7;
    		idBuffer[stepBufferIndex] = (iBufferFilter?Isdf:park1.Ds);
    		stepBufferIndex++;
        	if(PRE_BUFFER_LENGTH+BUFFER_LENGTH == stepBufferIndex){
        		stepRecord = 0;
        		stepBufferIndex = PRE_BUFFER_LENGTH;
        	}
    	}else{
    		preSpeedBuffer[preStepBufferIndex] = speed1.Speed;
    		preSpeedRefBuffer[preStepBufferIndex] = pid1_spd.Ref;
    		//preIqBuffer[preStepBufferIndex] = (iBufferFilter?Isqf:park1.Qs);
    		preIqBuffer[preStepBufferIndex] = AdcMirror.ADCRESULT7;
    		preIdBuffer[preStepBufferIndex] = (iBufferFilter?Isdf:park1.Ds);
    		preStepBufferIndex++;
    		if(PRE_BUFFER_LENGTH == preStepBufferIndex){
    			preStepBufferIndex = 0;
    		}
    	}
    }

    //Habilitacion de debilitamiento de campo en modo 3
	if (enable_weakening == 1 && ModoAD == 3) {
		IdRef = FieldWeakening(Speedf);
	}

    // Validación de datos ingresados por teclado
	if(SpeedRef > 0.52)
		SpeedRef = 0.52;
	if(SpeedRef < -0.52)
		SpeedRef = -0.52;
	if(IdRef > 0.313)
		IdRef = 0.313;
	if(IdRef < 0)
		IdRef = 0.01;
	


    rc1.TargetValue = SpeedRef;	//Carga referencia de velocidad al control de rampa
	RC_MACRO(rc1)				//Llama la macro para el cálculo de set point de frecuencia (Para RG), a la cuál llega en forma de rampa

    // ------------------------------------------------------------------------------
    //  Call the QEP macro (if incremental encoder used for speed sensing)
    //  Connect inputs of the SPEED_FR module and call the speed calculation macro
    // ------------------------------------------------------------------------------
        QEP_MACRO(qep1)
		angulo=1-qep1.ElecTheta;
        speed1.ElecTheta = angulo;
		speed1.DirectionQep = (int32)qep1.DirectionQep;
        SPEED_FR_MACRO(speed1)

    	Speedf = (speed1.Speed + Speed_1)*0.0004567174753 + 0.999086565*Speedf; //20Hz (speed1.Speed + Speed_1)*0.000182737 + 0.9996345259*Speedf;
    	Speed_1 = speed1.Speed;

// ------------------------------------------------------------------------------
//  Contador de para realizar ejecuciones de macro de manera óptima
// ------------------------------------------------------------------------------
   	if(Contador_Ejecuciones == 60)
   		Contador_Ejecuciones = 1;
   	else
   		Contador_Ejecuciones++;

// ------------------------------------------------------------------------------
//  Connect inputs of the RAMP GEN module and call the ramp generator macro
// ------------------------------------------------------------------------------
    rg1.Freq = rc1.SetpointValue;	//Carga el set point de frecuencia en forma de rampa (arranque suave)
	RG_MACRO(rg1)					//Llama la macro para el cálculo del ángulo del flujo (theta)(se usa únicamente cuando no existe el CURMOD)

// ------------------------------------------------------------------------------
//  Conecta entradas a rampas
// ------------------------------------------------------------------------------
    rc4.TargetValue = WindSpeed;	//Carga referencia de velocidad al control de rampa
	RC_MACRO(rc4)

    rc5.TargetValue = TurbineSpeed;	//Carga referencia de velocidad al control de rampa
	RC_MACRO(rc5)

    rc6.TargetValue = IdRef;	//Carga referencia de Id al control de rampa
	RC_MACRO(rc6)

// ------------------------------------------------------------------------------
//  Calcula una aleatoria para generar el patrón de viento
// ------------------------------------------------------------------------------
	if(Contador_Ejecuciones == 1)
	{
		patron_viento.Noise = AWGN_generator();
		patron_viento.WSpeed = rc4.SetpointValue*BASE_WIND_SPEED;
		WIND_PATTERN_MACRO(patron_viento)
	}

// ------------------------------------------------------------------------------
//  Connect inputs of the STATIC_CURVE module and call the static curve macro
// ------------------------------------------------------------------------------
	if(Contador_Ejecuciones % 10 == 0)
	{
		switch(staticCurveWSpeedSwitch)
		{
			case SW_CONST_W_SPD:
			static_curve1.WSpeed = rc4.SetpointValue;	//Se debe entrar con la velocidad del viento y la de la turbina para calcular el torque de salida
			break;
			case SW_PATTERN_W_SPD:
			static_curve1.WSpeed = patron_viento.Out;
			break;
		}

		switch(staticCurveTSpeedSwitch)
		{
			case SW_CONST_T_SPD:
			static_curve1.TSpeed = rc5.SetpointValue;			// Elijo velocidad de la turbina
			break;
			case SW_MEAS_T_SPD:
			static_curve1.TSpeed = _IQmpy(speed1.Speed,120.0/50.0);	//Esto se hace para normalizar
			break;
		}

		STATIC_CURVE_AD_MACRO(static_curve1)	// Hago el cálculo del torque de salida cada 10 ciclos
	}

	rc2.TargetValue = static_curve1.Torque;				// Como segunda opción, calcula el set point de torque utilizando una rampa
	RC_MACRO(rc2)

	if(SW_ENS_INEM)	//todo
	{
		if(speed1.Speed > 0.4)
		{
			TorqueRef = 0;
		}

		inertia_emulator1.TorqueRef = TorqueRef*TOR_NOM/BASE_TORQUE;
	}
	else
	{
	if(SW_WS_TS)
	{
		if(k == Kmax)
		{
			k=0;
			if((speed1.Speed < 0.01)&&(speed1.Speed > -0.01)){U3=1;}		//Evito división por cero
			else{Kmax=(1/_IQmpy(_IQabs(speed1.Speed),3))*3000;}//*100*gear Ratio;		// 1/(speed1.Speed*3*60)*6000 * "gear Ratio"  es la cantidad de periodos de muestreo por tercio de revolución
		}																	// 1seg, sobre la frecuencia en P.U. por 3 pulsos y por 60 para pasar a Hz de la frec. mecánica = al periodo
		else																// multiplico por la fecuencia de muestreo y por la constante de reducción ( >0 )
		{
			k++;
			if(k < (0.1*Kmax)){U3=k/(0.1*Kmax);}								//Tiempo de subida = al 10%
			else if(k < (0.9*Kmax)){U3=1.0;}									//80% en 1
			else {U3= 1- (k-0.9*Kmax)/(0.1*Kmax);}								//Tiempo de bajada = al 10%
		}
		U3=(U3+20)/21.0;													//Normalizo para tener un riple de torque cercano al 5%
		
		inertia_emulator1.TorqueRef = static_curve1.Torque*U3*TOR_NOM/BASE_TORQUE;		//Entro al emulador de inercia con el torque perturbado por efecto de wind shear y tower shadow
	}
	else
	{
		inertia_emulator1.TorqueRef = static_curve1.Torque*TOR_NOM/BASE_TORQUE; // Entro al emulador de inercia con el torque proveniente de la curva estática
	}
	}
    inertia_emulator1.SpeedF = _IQ(Speedf);					// Entro al emulador de inercia con la velocidad del motor FILTRADA
    inertia_emulator1.Speed = speed1.Speed;				// Entro al emulador de inercia con la velocidad del motor
    INERTIA_EMULATOR_MACRO(inertia_emulator1)			// Llama la macro para el cálculo de la referencia de torque del motor

// ------------------------------------------------------------------------------
//  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
//	Connect inputs of the CLARKE module and call the clarke transformation macro
// ------------------------------------------------------------------------------
// 1/4096=0.00024414. Resta 0.5 para eliminar la referencia de 1.5V y multiplica por 2 para lograr 1 a fondo de escala.
	clarke1.As=((AdcMirror.ADCRESULT1)*0.00024414-0.5)*2.0-IaOffset; // Phase U curr.
	clarke1.Bs=-((AdcMirror.ADCRESULT0)*0.00024414-0.5)*2.0-IbOffset-clarke1.As; // Phase V curr. (haciendo la diferencia obtenemos V)

	CLARKE_MACRO(clarke1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PARK module and call the park trans. macro
// ------------------------------------------------------------------------------
	park1.Alpha = clarke1.Alpha;
	park1.Beta  = clarke1.Beta;
	switch(iParkAngleSwitch){ //Selecciona de dónde tomo el ángulo del flujo rotórico
		case SW_RG:
			park1.Angle = rg1.Out; //SpeedRef
			break;
		case SW_CM:
			park1.Angle = cm1.Theta; //Modelo de corriente
			break;
	}
	park1.Sine=_IQsinPU(park1.Angle);
    park1.Cosine=_IQcosPU(park1.Angle);
	PARK_MACRO(park1)

// ------------------------------------------------------------------------------
//  Medición de temperatura del disipador
// ------------------------------------------------------------------------------
	Temperature = ((AdcMirror.ADCRESULT5)*0.00024414);

// ------------------------------------------------------------------------------
//  Versiones filtradas de Isq, Isd, Ief, Vdc y Speed
// ------------------------------------------------------------------------------
	Isqf= 0.000999 * park1.Qs + 0.999001 * Isqf;

	Isdf= 0.000999 * park1.Ds + 0.999001 * Isdf;

	Ieff=_IQmag(Isdf, Isqf)*BASE_CURRENT*0.707106;	//Corriente en P.U. por la base y sobre sqrt(2) para sacar la corriente eficaz

	Vdcf= 0.000999 * volt1.DcBusVolt + 0.999001 * Vdcf;

	TorqueHat = _IQmpy(_IQmpy(_IQmpy(_IQmpy(Isdf,Isqf),0.2859),2/3),BASE_TORQUE); //Tem=Isd*Isq*c*Lm/(1+SigmaR)*BASE_TORQUE
	
	TorqueHatGarciamod = _IQmpy(_IQmpy(Isdf,Isqf),57.867); // Tem = c*np*(Lo/(1+Sigma_r))*BASE_TORQUE*Isd*Isq con c=3/2 y np=2

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_REG3 module and call the PID SPD controller macro
// ------------------------------------------------------------------------------
    if(_IQabs(IdRef) < 0.01){IdRef = 0.01;} //evito la división por cero al adaptar el torque a la magnetización
    if (SpeedLoopCount==SpeedLoopPrescaler)
    {
    	// Parte de la protección de sobrevelocidad para BuildLevel mayores a 3
    	if(ModoAD > 3)
    	{
    		pid1_spd.Ref = SpeedRef;
			pid1_spd.OutMax = _IQdiv(IsqFactor,IdRef);
			pid1_spd.OutMin = _IQdiv(-IsqFactor,IdRef);
    	}
		else
		{
			pid1_spd.Ref = rc1.SetpointValue;
		}

      pid1_spd.Fdb = speed1.Speed;
	  PID_MACRO(pid1_spd)
      SpeedLoopCount=1;
    }
    else SpeedLoopCount++;

	if(spdBlocker)
	{
		pid1_spd.Ui = 0;
		pid1_spd.Out = pid1_iq.Ref;
	}

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_REG3 module and call the PID IQ controller macro
// ------------------------------------------------------------------------------


	switch(pidIqRefSwitch)
    {							//Selecciona de dónde toma la referencia el PI de Iq
    	case SW_IQ_REF:
    		pid1_iq.Ref = IqRef;
    		IQSWITCH_FLAG=-1;
    		spdBlocker=1;
    		break;
    	case SW_PID_SPD:
    		pid1_iq.Ref = pid1_spd.Out;
    		IQSWITCH_FLAG=-1;
    		spdBlocker=0;
    		break;
    	case SW_TORQUE_RAMP:
			if(_IQabs(_IQmpy(rc2.SetpointValue,_IQdiv(IsqFactor,IdRef))) < _IQabs(pid1_spd.Out))
			{   
				pid1_iq.Ref = _IQmpy(rc2.SetpointValue,_IQdiv(IsqFactor,IdRef));
			}
			else
			{
				pid1_iq.Ref = pid1_spd.Out;                                                                                                                                    
			}
    		//  Tnom =9.55Nm   IsqFactor=9.55/c*np*(Lo/(1+Sigma_r))*BASE_TORQUE con c=3/2 y np=2 <--Sacado del paper de García y modificado c y  n
    		//Con esto logramos una salida adaptable dinámica de acuerdo a nuestra corriente de magnetización.
    		IQSWITCH_FLAG=-1;
    		spdBlocker=0;
    	   	break;
    	case SW_INERTIA_EMULATOR:
    		if(IQSWITCH_FLAG != 1)
    		{	// Conecta la salida del emulador de inercia al PID de Iq mediante una rampa hasta que el valor se encuentra a
    			// un 5% del valor final, únicamente la primera vez
    			rc3.TargetValue = _IQmpy(inertia_emulator1.TorqueOut,_IQdiv(IsqFactor,IdRef));// Tnom =9.55Nm   IsqFactor=9.55/c*np*(Lo/(1+Sigma_r))*BASE_TORQUE con c=3/2 y np=2 <--Sacado del paper de García y modificado c y  n
    			RC_MACRO(rc3);
    			if((rc3.SetpointValue<=(1.05 * inertia_emulator1.TorqueOut)) && (rc3.SetpointValue >= (0.95 * inertia_emulator1.TorqueOut)))
    			{
    				IQSWITCH_FLAG=1;
    			}
    		}
    		else
    		{
    			if(_IQabs(_IQmpy(inertia_emulator1.TorqueOut,_IQdiv(IsqFactor,IdRef))) < _IQabs(pid1_spd.Out) )
				{   
					pid1_iq.Ref = _IQmpy(inertia_emulator1.TorqueOut,_IQdiv(IsqFactor,IdRef));
				}
				else
				{
					pid1_iq.Ref = pid1_spd.Out;                                                                                                                                    
				}
				// Tnom =9.55Nm   IsqFactor=9.55/c*np*(Lo/(1+Sigma_r))*BASE_TORQUE con c=3/2 y np=2 <--Sacado del paper de García y modificado c y  n
    		}
    		spdBlocker=0;
    		break;
    }

    // Protección en caso que haya sobrevelocidad cuando estamos en algún BuildLevel superior a 3
    if(_IQabs(pid1_spd.Out) < _IQabs(pid1_iq.Ref))
	{
    	pid1_iq.Ref = pid1_spd.Out;
	}

	pid1_iq.Fdb = park1.Qs;
	PID_MACRO(pid1_iq)

	if(iqBlocker)
	{
		pid1_iq.Ui = 0;
		pid1_iq.Out = ipark1.Qs;
	}

// ------------------------------------------------------------------------------
//  Connect inputs of the PID_REG3 module and call the PID ID controller macro
// ------------------------------------------------------------------------------

    //pid1_id.Ref = rc6.SetpointValue;// QUITAMOS LA RAMPA
    pid1_id.Ref =  IdRef; //HABILITAMOS ESCALON DE SET POINT DE CORRIENTE EN D
	pid1_id.Fdb = park1.Ds;
	PID_MACRO(pid1_id)

	if(idBlocker)
	{
		pid1_id.Ui = 0;
		pid1_id.Out = ipark1.Ds;
	}

	//----------------------------------------------------------------------------------
	// TOMO LAS SALIDAS DE pid1_id y de pid1_iq. CALCULO EL MODULO DEL VECTOR DE TENSION:
	// SQRT(pid1_id.Out^2 + pid1_iq.Out^2)
	// SI EL MODULO ES MAYOR A 1 SIGNIFICA QUE SE INTENTARA SOBREMODULAR, COSA QUE LA SVGEN MACRO
	// NO PUEDE HACER. ENTONCES REALIZO UN ESCALAMIENTO PARA LOGRAR MODULO MAXIMO 1.
	//----------------------------------------------------------------------------------
		modulo = _IQmag(pid1_id.Out,pid1_iq.Out);
		if(modulo>_IQ(0.99)){
		    Vd=_IQdiv(pid1_id.Out,_IQmpy(modulo,_IQ(1.01)));
		    Vq=_IQdiv(pid1_iq.Out,_IQmpy(modulo,_IQ(1.01)));
		}
		else{
		    Vd=pid1_id.Out;
		    Vq=pid1_iq.Out;
		}
// ------------------------------------------------------------------------------
//	Connect inputs of the INV_PARK module and call the inverse park trans. macro
// ------------------------------------------------------------------------------
    switch(iParkDsSwitch)
    {
    	case SW_VD_TESTING:
    		ipark1.Ds = VdTesting;
    		idBlocker=1;
    		break;
    	case SW_PID_ID:
    		ipark1.Ds = Vd;	//Salida del PI de Id
    		idBlocker=0;
    		break;
    }

    switch(iParkQsSwitch)
    {
    	case SW_VQ_TESTING:
    		ipark1.Qs = VqTesting;
    		iqBlocker=1;
    		break;
    	case SW_PID_IQ:
    		ipark1.Qs = Vq;	//Salida del PI de Id
    		iqBlocker=0;
    		break;
    }

	ipark1.Angle = park1.Angle;
	ipark1.Sine  = _IQsinPU(ipark1.Angle);
	ipark1.Cosine= _IQcosPU(ipark1.Angle);
	IPARK_MACRO(ipark1)

// ------------------------------------------------------------------------------
//  Connect inputs of the CURMOD module and call the current model macro
// ------------------------------------------------------------------------------
    cm1.IDs = park1.Ds;
    cm1.IQs = park1.Qs;
	cm1.Wr = speed1.Speed;
	CUR_MOD_MACRO(cm1)

// ------------------------------------------------------------------------------
//  Connect inputs of the VOLT_CALC module and call the phase voltage calc. macro
// ------------------------------------------------------------------------------
	volt1.DcBusVolt = ((AdcMirror.ADCRESULT3)*0.00024414); // DC Bus voltage meas.// (ADCmeas(q12)/2^12)*(3.0V/3.3V)
    volt1.MfuncV1 = svgen_dq1.Ta;
    volt1.MfuncV2 = svgen_dq1.Tb;
    volt1.MfuncV3 = svgen_dq1.Tc;
    VOLT_MACRO(volt1)

// ------------------------------------------------------------------------------
//  Protecciones GRAL
// ------------------------------------------------------------------------------
	if(Vdcf>=0.999 || Ieff>=40 || speed1.Speed>=0.55 || speed1.Speed<=-0.55 || clarke1.Alpha>=1 || STOP==1 ||GpioDataRegs.GPADAT.bit.GPIO6 == 1 /*OverTemp*/ || GpioDataRegs.GPADAT.bit.GPIO26 == 1 /*OCP*/ || GpioDataRegs.GPADAT.bit.GPIO14 == 1 /*DESAT*/)
	{	 //Vdcd .5 clarke1.Alpha 0.4 todo DESCOMENTAR OverTemp
		if(!STOP)
		{
			if(speed1.Speed>=0.55 || speed1.Speed<=-0.55){
				SPEED_FLAG=1;
			}
			else{
				SPEED_FLAG=0;
			}

			VDC_FLAG=Vdcf>=0.999?1:0;
			CLARKE_FLAG=clarke1.Alpha>=1?1:0;
			IEFF_FLAG=Ieff>=40?1:0;
			OVERTEMP_FLAG=GpioDataRegs.GPADAT.bit.GPIO6==1?1:0;
			OCP_FLAG=GpioDataRegs.GPADAT.bit.GPIO26==1?1:0;
			DESAT_FLAG=GpioDataRegs.GPADAT.bit.GPIO14==1?1:0;

		}
		STOP=1;	//Flag de parada.
/*
	//ABRE LA PRIMERA RAMA DE IGBTs
		//Load immediately (the active register is directly
		//accessed by the CPU and is not loaded from the shadow register).
		EPwm1Regs.AQSFRC.bit.RLDCSF = 3;

		//Clear (low) / Clear (low)
		EPwm1Regs.AQSFRC.bit.ACTSFA = 1;
		EPwm1Regs.AQSFRC.bit.ACTSFB = 1;

		//Initiates a single software forced event
		//Initiates a single software forced event
		EPwm1Regs.AQSFRC.bit.OTSFA = 1;
		EPwm1Regs.AQSFRC.bit.OTSFB = 1;

		//Forces a continuous low on output A
		//Forces a continuous low on output B
		EPwm1Regs.AQCSFRC.bit.CSFA = 1;
		EPwm1Regs.AQCSFRC.bit.CSFB = 1;

		//Dead-band generation is bypassed for both output signals.
		//In this mode, both the EPWMxA and EPWMxB output signals
		//from the action-qualifier are passed directly to the PWM-chopper submodule.
		EPwm1Regs.DBCTL.bit.OUT_MODE = 0;

	//ABRE LA SEGUNDA RAMA DE IGBTs
		//Load immediately (the active register is directly
		//accessed by the CPU and is not loaded from the shadow register).
		EPwm2Regs.AQSFRC.bit.RLDCSF = 3;

		//Clear (low) / Clear (low)
		EPwm2Regs.AQSFRC.bit.ACTSFA = 1;
		EPwm2Regs.AQSFRC.bit.ACTSFB = 1;

		//Initiates a single software forced event
		//Initiates a single software forced event
		EPwm2Regs.AQSFRC.bit.OTSFA = 1;
		EPwm2Regs.AQSFRC.bit.OTSFB = 1;

		//Forces a continuous low on output A
		//Forces a continuous low on output B
		EPwm2Regs.AQCSFRC.bit.CSFA = 1;
		EPwm2Regs.AQCSFRC.bit.CSFB = 1;

		//Dead-band generation is bypassed for both output signals.
		//In this mode, both the EPWMxA and EPWMxB output signals
		//from the action-qualifier are passed directly to the PWM-chopper submodule.
		EPwm2Regs.DBCTL.bit.OUT_MODE = 0;

	//ABRE LA TERCERA RAMA DE IGBTs
		//Load immediately (the active register is directly
		//accessed by the CPU and is not loaded from the shadow register).
		EPwm3Regs.AQSFRC.bit.RLDCSF = 3;

		//Clear (low) / Clear (low)
		EPwm3Regs.AQSFRC.bit.ACTSFA = 1;
		EPwm3Regs.AQSFRC.bit.ACTSFB = 1;

		//Initiates a single software forced event
		//Initiates a single software forced event
		EPwm3Regs.AQSFRC.bit.OTSFA = 1;
		EPwm3Regs.AQSFRC.bit.OTSFB = 1;

		//Forces a continuous low on output A
		//Forces a continuous low on output B
		EPwm3Regs.AQCSFRC.bit.CSFA = 1;
		EPwm3Regs.AQCSFRC.bit.CSFB = 1;

		//Dead-band generation is bypassed for both output signals.
		//In this mode, both the EPWMxA and EPWMxB output signals
		//from the action-qualifier are passed directly to the PWM-chopper submodule.
		EPwm3Regs.DBCTL.bit.OUT_MODE = 0;
*/
		TZ_Protection();
	}

// ------------------------------------------------------------------------------
//  Connect inputs of the SVGEN_DQ module and call the space-vector gen. macro
// ------------------------------------------------------------------------------
  	svgen_dq1.Ualpha = ipark1.Alpha;
 	svgen_dq1.Ubeta = ipark1.Beta;
  	SVGEN_MACRO(svgen_dq1)

// ------------------------------------------------------------------------------
//  Connect inputs of the PWM_DRV module and call the PWM signal generation macro
// ------------------------------------------------------------------------------
    pwm1.MfuncC1 = _IQtoQ15(svgen_dq1.Ta); // MfuncC1 is in Q15
    pwm1.MfuncC2 = _IQtoQ15(svgen_dq1.Tb); // MfuncC2 is in Q15
    pwm1.MfuncC3 = _IQtoQ15(svgen_dq1.Tc); // MfuncC3 is in Q15
	PWM_MACRO(pwm1)						   // Calculate the new PWM compare values

	EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;	// PWM 1A - PhaseA
	EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;	// PWM 2A - PhaseB
	EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;	// PWM 3A - PhaseC

	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; //VOLINER PATCH CRACK KEYGEN SERVICE PACK

#if (DSP2803x_DEVICE_H==1)||(DSP280x_DEVICE_H==1)||(DSP2833x_DEVICE_H==1)
// Enable more interrupts from this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

// Acknowledge interrupt to recieve more interrupts from PIE group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
#endif
}


//Protección por desaturación
interrupt void DesatISR(void)
{
	TZ_Protection();
}


//Proteccion por TRIPZONE
void TZ_Protection(void)
{
//Requiero permiso para modificar estos registros
EALLOW;
//Fuerzo falla por software en los 3 PWM
	EPwm1Regs.TZFRC.bit.OST=1;
	EPwm2Regs.TZFRC.bit.OST=1;
	EPwm3Regs.TZFRC.bit.OST=1;
EDIS;
	TRIP=1;
}


//Funcion que resetea la interrupcion del TripZone y permite al equipo continuar trabajando
void TZ_Clear(void)
{
//Requiero permiso para modificar estos registros
EALLOW;
//Fuerzo reset y deshabilito la proteccion
	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;
	TRIP=0;
EDIS;
}


float AWGN_generator()
{	//Box-Muller Method: The Box-Muller method uses the technique of inverse transformation to turn two uniformly distributed randoms, U1 and U2, into one unit normal random, X.
	/* Generates additive white Gaussian Noise samples with zero mean and a standard deviation of 1. */
	float cov=0.0008;
	float Tmuestra=0.01;
	float U1;
	float U2;
	float AWGN;

	U1=0;
	while (U1==0)
	{
	   U1=rand()/32767.0;
	}
	U2=rand()/32767.0;
	AWGN=sqrt(-2.0*log(U1))*cos(2.0*PI*U2)*sqrt(cov)/sqrt(Tmuestra);
	return AWGN;
}// end AWGN_generator

_iq FieldWeakening(_iq speed) {
	_iq direct_current = 0.0;
	int i = 0;
	if (speed <= 0.4166) {
		direct_current = 0.2725;
	}
	else if (speed > 0.4166 && speed < 0.5) {
		for(i=0;i<10;i++){
			if(speed == speeds[i]){
				direct_current=currents[i];
				break;
			}
			else{
				if(speed > speeds[i] && speed < speeds[i+1] && i < 9){
					direct_current= ((currents[i+1] - currents[i])/(speeds[i+1]-speeds[i])*(speed-speeds[i]))+currents [i];
				}
			}

		}
	}
	else{
		direct_current = currents[9];
	}
	return direct_current;
}
