/*
 * parameter_list.c
 *
 *  Created on: Aug 2, 2023
 *      Author: Wiktor
 */

#include "parameter_list.h"
#include <string.h>
#include <math.h>

HAL_StatusTypeDef limiter(parameter_t * par, void * pValue, void * pReturnValue);
HAL_StatusTypeDef prepare_received_data(parameter_t * par, uint32_t * pReceivedData, void * pReturnValue);

parameter_t parameter_list[]={
		{.number=1,.ModbusAddress=1,.CANAddress=0x606C,.name={"Actual speed"},.shortName={"Act spd"},.description={"Actual motor speed"},.WriteAllowed=0,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-10000.0f,.maxValue=10000.0f,.defaultValue=0.0f},
		{.number=2,.ModbusAddress=2,.CANAddress=1111,.name={"Apparent current"},.shortName={"I mot"},.description={"Actual motor RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=3,.ModbusAddress=3,.CANAddress=1111,.name={"Magnetizing current"},.shortName={"I mag"},.description={"Magnetizing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=4,.ModbusAddress=4,.CANAddress=1111,.name={"Torque current"},.shortName={"I trq"},.description={"Torque-producing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=5,.ModbusAddress=5,.CANAddress=1111,.name={"Motor torque"},.shortName={"Mot trq"},.description={"Actaul motor torque in %"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=6,.ModbusAddress=6,.CANAddress=1111,.name={"Actual position L"},.shortName={"Act posL"},.description={"Actual rotor position low word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-32768.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=7,.ModbusAddress=7,.CANAddress=0x6064,.name={"Actual position H"},.shortName={"Act posH"},.description={"Actual axis position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-32768.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=8,.ModbusAddress=8,.CANAddress=1111,.name={"Target position L"},.shortName={"Tht posL"},.description={"Target axis position high word"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-32768.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=9,.ModbusAddress=9,.CANAddress=1111,.name={"Target position H"},.shortName={"Tgt posH"},.description={"Target axis position high word"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-32768.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=10,.ModbusAddress=10,.CANAddress=1111,.name={"Motor voltage"},.shortName={"U mot"},.description={"Actual motor RMS voltage"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=11,.ModbusAddress=11,.CANAddress=1111,.name={"DC bus voltage"},.shortName={"DC bus"},.description={"Actual voltage in intermediate circuit"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=12,.ModbusAddress=12,.CANAddress=1111,.name={"Motor thermal load"},.shortName={"Mot I2t"},.description={"Motor thermal calculated from output current"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=20,.ModbusAddress=20,.CANAddress=1111,.name={"Actual error"},.shortName={"Act Err"},.description={"Actual error number"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=21,.ModbusAddress=21,.CANAddress=1111,.name={"Error history 1"},.shortName={"Err1"},.description={"Error history 1(latest)"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=22,.ModbusAddress=22,.CANAddress=1111,.name={"Error history 2"},.shortName={"Err2"},.description={"Error history 2"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=23,.ModbusAddress=23,.CANAddress=1111,.name={"Error history 3"},.shortName={"Err3"},.description={"Error history 3"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=24,.ModbusAddress=24,.CANAddress=1111,.name={"Error history 4"},.shortName={"Err4"},.description={"Error history 4"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=25,.ModbusAddress=25,.CANAddress=1111,.name={"Error history 5"},.shortName={"Err5"},.description={"Error history 5"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=26,.ModbusAddress=26,.CANAddress=1111,.name={"Error history 6"},.shortName={"Err6"},.description={"Error history 6"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=27,.ModbusAddress=27,.CANAddress=1111,.name={"Error history 7"},.shortName={"Err7"},.description={"Error history 7"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=28,.ModbusAddress=28,.CANAddress=1111,.name={"Error history 8"},.shortName={"Err8"},.description={"Error history 8"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=29,.ModbusAddress=29,.CANAddress=1111,.name={"Error history 9"},.shortName={"Err9"},.description={"Error history 9"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=30,.ModbusAddress=30,.CANAddress=0x6041,.name={"Status register"},.shortName={"Sts reg"},.description={"Status register"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=31,.ModbusAddress=31,.CANAddress=0x6040,.name={"Control register"},.shortName={"Ctl reg"},.description={"Control register"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=32,.ModbusAddress=32,.CANAddress=0x60FF,.name={"Target speed"},.shortName={"Tgt spd"},.description={"Target speed"},.WriteAllowed=1,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-30000.0f,.maxValue=30000.0f,.defaultValue=0.0f},
		{.number=33,.ModbusAddress=33,.CANAddress=0x6071,.name={"Target torque"},.shortName={"Tgt trq"},.description={"Target torque"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=34,.ModbusAddress=34,.CANAddress=1111,.name={"Field target"},.shortName={"Fld trq"},.description={"Target field"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=35,.ModbusAddress=35,.CANAddress=1111,.name={"Open loop freq"},.shortName={"OL freq"},.description={"Actual output frequency in uf/open loop mode"},.WriteAllowed=0,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},
		{.number=36,.ModbusAddress=36,.CANAddress=1111,.name={"Target open loop freq"},.shortName={"Freq tgt"},.description={"Output frequency target in uf/open loop mode"},.WriteAllowed=1,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},
		{.number=37,.ModbusAddress=37,.CANAddress=1111,.name={"Target open loop volt"},.shortName={"Volt tgt"},.description={"Output voltage target in manual"},.WriteAllowed=1,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=480.0f,.defaultValue=0.0f},

		{.number=40,.ModbusAddress=40,.CANAddress=1111,.name={"Encoder pulse"},.shortName={"Enc puls"},.description={"One turn absolute encoder pulse"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=41,.ModbusAddress=41,.CANAddress=1111,.name={"Stator Electric Angle"},.shortName={"Fld ang"},.description={"Actual stator field electric angle"},.WriteAllowed=1,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=359.9f,.defaultValue=0.0f},
		{.number=42,.ModbusAddress=42,.CANAddress=1111,.name={"Rotor Electric Angle"},.shortName={"Rot ang"},.description={"Actual rotor field electric angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=360.0f,.defaultValue=0.0f},
		{.number=43,.ModbusAddress=43,.CANAddress=1111,.name={"Torque angle"},.shortName={"Trq ang"},.description={"Difference between stator and rotor angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=-180.0f,.maxValue=180.0f,.defaultValue=0.0f},
		{.number=44,.ModbusAddress=44,.CANAddress=1111,.name={"U current"},.shortName={"I U"},.description={"Actual U phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=45,.ModbusAddress=45,.CANAddress=1111,.name={"V current"},.shortName={"I V"},.description={"Actual V phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=46,.ModbusAddress=46,.CANAddress=1111,.name={"W current"},.shortName={"I W"},.description={"Actual W phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},

		{.number=100,.ModbusAddress=100,.CANAddress=0x6060,.name={"Motor control mode"},.shortName={"Ctl mode"},.description={"Motion control mode"},.WriteAllowed=2,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-5.0f,.maxValue=7.0f,.defaultValue=1.0f},
		{.number=101,.ModbusAddress=101,.CANAddress=1111,.name={"Save EEPROM"},.shortName={"Save par"},.description={"Save parameters in non-volatile memory"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=1.0f,.defaultValue=0.0f},

		{.number=120,.ModbusAddress=120,.CANAddress=1111,.name={"Factor Numerator"},.shortName={"Fct num"},.description={"Encoder scaling factor - numerator"},.WriteAllowed=2,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=1.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=121,.ModbusAddress=121,.CANAddress=1111,.name={"Factor Denominator"},.shortName={"Fct den"},.description={"Encoder scaling factor - denominator"},.WriteAllowed=2,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=1.0f,.maxValue=65535.0f,.defaultValue=0.0f},


		{.number=200,.ModbusAddress=200,.CANAddress=1111,.name={"Feedback type"},.shortName={"Fbk typ"},.description={"Rotor position feedback type"},.WriteAllowed=2,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=5.0f,.defaultValue=1.0f},
		{.number=201,.ModbusAddress=201,.CANAddress=1111,.name={"Feedback offset"},.shortName={"Fbk ofs"},.description={"Difference of feedback el angle and rotor"},.WriteAllowed=1,.precision=2,.unit="deg",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-180.0f,.maxValue=180.0f,.defaultValue=0.0f},
		{.number=202,.ModbusAddress=202,.CANAddress=1111,.name={"Encoder resolution"},.shortName={"Enc res"},.description={"Points generated by encoder for 1 rotation"},.WriteAllowed=2,.precision=0,.unit="pls",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=16000.0f,.defaultValue=2048.0f},
		{.number=203,.ModbusAddress=203,.CANAddress=1111,.name={"Feedback polarity"},.shortName={"Fbk pol"},.description={"Switches direction of encoder output"},.WriteAllowed=2,.precision=2,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=1.0f,.defaultValue=0.0f},

		{.number=210,.ModbusAddress=210,.CANAddress=1111,.name={"Motor pole pairs"},.shortName={"Mot pp"},.description={"Number of motor pole pairs"},.WriteAllowed=2,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=1.0f,.maxValue=24.0f,.defaultValue=4.0f},
		{.number=211,.ModbusAddress=211,.CANAddress=1111,.name={"Motor nominal current"},.shortName={"Mot Inom"},.description={"Nominal RMS current of motor"},.WriteAllowed=2,.precision=2,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.5f,.maxValue=15.0f,.defaultValue=4.8f},
		{.number=212,.ModbusAddress=212,.CANAddress=1111,.name={"Motor overload current"},.shortName={"Mot Imax"},.description={"Max RMS current of motor"},.WriteAllowed=2,.precision=2,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.5f,.maxValue=15.0f,.defaultValue=4.8f},
		{.number=213,.ModbusAddress=213,.CANAddress=1111,.name={"Motor nominal voltage"},.shortName={"Mot Unom"},.description={"Nominal RMS voltage of motor"},.WriteAllowed=2,.precision=1,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=12.0f,.maxValue=480.0f,.defaultValue=230.0f},
		{.number=214,.ModbusAddress=214,.CANAddress=1111,.name={"Motor base frequency"},.shortName={"Mot fnom"},.description={"Nominal frequency of motor"},.WriteAllowed=2,.precision=2,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.5f,.maxValue=650.0f,.defaultValue=200.0f},
		{.number=215,.ModbusAddress=215,.CANAddress=1111,.name={"Motor nominal torque"},.shortName={"Nom trq"},.description={"Nominal motor torque"},.WriteAllowed=2,.precision=1,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.1f,.maxValue=650.0f,.defaultValue=2.5f},
		{.number=216,.ModbusAddress=216,.CANAddress=1111,.name={"Motor nominal speed"},.shortName={"Nom RPM"},.description={"Nominal RPM of motor"},.WriteAllowed=2,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=1.0f,.minValue=1.0f,.maxValue=60000.0f,.defaultValue=3000.0f},
		{.number=217,.ModbusAddress=217,.CANAddress=1111,.name={"Motor max speed"},.shortName={"Max RPM"},.description={"RPM limit of motor"},.WriteAllowed=2,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=1.0f,.minValue=5.0f,.maxValue=60000.0f,.defaultValue=4000.0f},
		{.number=218,.ModbusAddress=218,.CANAddress=1111,.name={"Motor Rs"},.shortName={"Mot Rs"},.description={"Phase-phase resistance of motor"},.WriteAllowed=2,.precision=3,.unit="Ohm",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.001f,.minValue=0.001f,.maxValue=60.0f,.defaultValue=0.65f},
		{.number=219,.ModbusAddress=219,.CANAddress=1111,.name={"Motor Ls"},.shortName={"Mot Ls"},.description={"Phase-phase inductance of motor"},.WriteAllowed=2,.precision=2,.unit="mH",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.5f,.maxValue=200.0f,.defaultValue=4.8f},
		{.number=220,.ModbusAddress=220,.CANAddress=1111,.name={"Motor Back-EMF constant"},.shortName={"Mot EMF"},.description={"Motor back-emf in V/1000RPM"},.WriteAllowed=2,.precision=1,.unit="Kv",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=1.0f,.maxValue=650.0f,.defaultValue=29.0f},
		{.number=221,.ModbusAddress=221,.CANAddress=1111,.name={"Motor inertia"},.shortName={"Mot Inr"},.description={"Motor inertia kgcm2 "},.WriteAllowed=2,.precision=2,.unit="kgc",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.05f,.maxValue=650.0f,.defaultValue=0.6f},

		{.number=300,.ModbusAddress=300,.CANAddress=0x6083,.name={"Acceleration ramp"},.shortName={"Acc ramp"},.description={"Motor acceleration ramp s/1000RPM"},.WriteAllowed=1,.precision=2,.unit="s",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.0f,.maxValue=650.0f,.defaultValue=3.0f},
		{.number=301,.ModbusAddress=301,.CANAddress=0x6084,.name={"Deceleration ramp"},.shortName={"Dec ramp"},.description={"Motor deceleration ramp s/1000RPM"},.WriteAllowed=1,.precision=2,.unit="s",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.0f,.maxValue=650.0f,.defaultValue=3.0f},
		{.number=302,.ModbusAddress=302,.CANAddress=1111,.name={"Speed limit positive"},.shortName={"Spd lim+"},.description={"Speed limit in positive direction"},.WriteAllowed=1,.precision=0,.unit="RPM",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65000.0f,.defaultValue=4000.0f},
		{.number=303,.ModbusAddress=303,.CANAddress=1111,.name={"Speed limit negative"},.shortName={"Spd lim-"},.description={"Speed limit in negative direction"},.WriteAllowed=1,.precision=0,.unit="RPM",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65000.0f,.defaultValue=4000.0f},

		{.number=310,.ModbusAddress=310,.CANAddress=1111,.name={"Position loop P gain"},.shortName={"Pos p"},.description={"Proportional gain of positioning loop"},.WriteAllowed=1,.precision=3,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.001f,.minValue=0.0f,.maxValue=6500.0f,.defaultValue=2.0f},
		{.number=311,.ModbusAddress=311,.CANAddress=1111,.name={"Position loop I time"},.shortName={"Pos i"},.description={"Integration time of positioning loop"},.WriteAllowed=1,.precision=1,.unit="ms",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=6500.0f,.defaultValue=0.0f},

		{.number=320,.ModbusAddress=320,.CANAddress=1111,.name={"Speed loop P gain"},.shortName={"Speed p"},.description={"Proportional gain of speed control loop"},.WriteAllowed=1,.precision=3,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.001f,.minValue=0.0f,.maxValue=65.0f,.defaultValue=0.029f},
		{.number=321,.ModbusAddress=321,.CANAddress=1111,.name={"Speed loop I time"},.shortName={"Speed i"},.description={"Integration time of speed control loop"},.WriteAllowed=1,.precision=1,.unit="ms",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=650.0f,.defaultValue=0.1f},
		{.number=323,.ModbusAddress=323,.CANAddress=1111,.name={"Speed filter tau"},.shortName={"Spd fltr"},.description={"Time constant of speed 1st order filter"},.WriteAllowed=1,.precision=0,.unit="ms",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=6500.0f,.defaultValue=1.0f},

		{.number=330,.ModbusAddress=330,.CANAddress=1111,.name={"Current loop P gain"},.shortName={"Curr p"},.description={"Proportional gain of current control loop"},.WriteAllowed=1,.precision=2,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=0.0f,.maxValue=650.0f,.defaultValue=0.03f},
		{.number=331,.ModbusAddress=331,.CANAddress=1111,.name={"Current loop I time"},.shortName={"Curr i"},.description={"Integration time of current control loop"},.WriteAllowed=1,.precision=1,.unit="",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=6500.0f,.defaultValue=3.0f},

};

parameter_t monitor_list[]={ //parameters shown in monitor menu
		{.number=1,.ModbusAddress=1,.CANAddress=1111,.name={"Actual speed"},.shortName={"Act spd"},.description={"Actual motor speed"},.WriteAllowed=0,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-10000.0f,.maxValue=10000.0f,.defaultValue=0.0f},
		{.number=2,.ModbusAddress=2,.CANAddress=1111,.name={"Apparent current"},.shortName={"I mot"},.description={"Actual motor RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=3,.ModbusAddress=3,.CANAddress=1111,.name={"Magnetizing current"},.shortName={"I mag"},.description={"Magnetizing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=4,.ModbusAddress=4,.CANAddress=1111,.name={"Torque current"},.shortName={"I trq"},.description={"Torque-producing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=5,.ModbusAddress=5,.CANAddress=1111,.name={"Motor torque"},.shortName={"Mot trq"},.description={"Actaul motor torque in %"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=6,.ModbusAddress=6,.CANAddress=1111,.name={"Actual position L"},.shortName={"Act posL"},.description={"Actual rotor position low word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=7,.ModbusAddress=7,.CANAddress=1111,.name={"Actual position H"},.shortName={"Act posH"},.description={"Actual axis position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-32767.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=8,.ModbusAddress=8,.CANAddress=1111,.name={"Target position L"},.shortName={"Tht posL"},.description={"Actual rotor position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=9,.ModbusAddress=9,.CANAddress=1111,.name={"Target position H"},.shortName={"Tgt posH"},.description={"Target axis position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-32767.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=10,.ModbusAddress=10,.CANAddress=1111,.name={"Motor voltage"},.shortName={"U mot"},.description={"Actual motor RMS voltage"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=11,.ModbusAddress=11,.CANAddress=1111,.name={"DC bus voltage"},.shortName={"DC bus"},.description={"Actual voltage in intermediate circuit"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=12,.ModbusAddress=12,.CANAddress=1111,.name={"Motor thermal load"},.shortName={"Mot I2t"},.description={"Motor thermal calculated from output current"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=20,.ModbusAddress=20,.CANAddress=1111,.name={"Actual error"},.shortName={"Act Err"},.description={"Actual error number"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=21,.ModbusAddress=21,.CANAddress=1111,.name={"Error history 1"},.shortName={"Err1"},.description={"Error history 1(latest)"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=22,.ModbusAddress=22,.CANAddress=1111,.name={"Error history 2"},.shortName={"Err2"},.description={"Error history 2"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=23,.ModbusAddress=23,.CANAddress=1111,.name={"Error history 3"},.shortName={"Err3"},.description={"Error history 3"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=24,.ModbusAddress=24,.CANAddress=1111,.name={"Error history 4"},.shortName={"Err4"},.description={"Error history 4"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=25,.ModbusAddress=25,.CANAddress=1111,.name={"Error history 5"},.shortName={"Err5"},.description={"Error history 5"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=26,.ModbusAddress=26,.CANAddress=1111,.name={"Error history 6"},.shortName={"Err6"},.description={"Error history 6"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=27,.ModbusAddress=27,.CANAddress=1111,.name={"Error history 7"},.shortName={"Err7"},.description={"Error history 7"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=28,.ModbusAddress=28,.CANAddress=1111,.name={"Error history 8"},.shortName={"Err8"},.description={"Error history 8"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=29,.ModbusAddress=29,.CANAddress=1111,.name={"Error history 9"},.shortName={"Err9"},.description={"Error history 9"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=30,.ModbusAddress=30,.CANAddress=1111,.name={"Status register"},.shortName={"Sts reg"},.description={"Status register"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=31,.ModbusAddress=31,.CANAddress=1111,.name={"Control register"},.shortName={"Ctl reg"},.description={"Control register"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=32,.ModbusAddress=32,.CANAddress=1111,.name={"Target speed"},.shortName={"Tgt spd"},.description={"Target speed"},.WriteAllowed=1,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplierMB=1.0f,.minValue=-30000.0f,.maxValue=30000.0f,.defaultValue=0.0f},
		{.number=33,.ModbusAddress=33,.CANAddress=1111,.name={"Target torque"},.shortName={"Tgt trq"},.description={"Target torque"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=34,.ModbusAddress=34,.CANAddress=1111,.name={"Field torque"},.shortName={"Fld trq"},.description={"Target field"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=35,.ModbusAddress=35,.CANAddress=1111,.name={"Open loop freq"},.shortName={"OL freq"},.description={"Actual output frequency in uf/open loop mode"},.WriteAllowed=0,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},
		{.number=36,.ModbusAddress=36,.CANAddress=1111,.name={"Target open loop freq"},.shortName={"Freq tgt"},.description={"Output frequency target in uf/open loop mode"},.WriteAllowed=1,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},

		{.number=40,.ModbusAddress=40,.CANAddress=1111,.name={"Encoder pulse"},.shortName={"Enc puls"},.description={"One turn absolute encoder pulse"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplierMB=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=41,.ModbusAddress=41,.CANAddress=1111,.name={"Stator Electric Angle"},.shortName={"Fld ang"},.description={"Actual stator field electric angle"},.WriteAllowed=1,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=359.9f,.defaultValue=0.0f},
		{.number=42,.ModbusAddress=42,.CANAddress=1111,.name={"Rotor Electric Angle"},.shortName={"Rot ang"},.description={"Actual rotor field electric angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=0.0f,.maxValue=360.0f,.defaultValue=0.0f},
		{.number=43,.ModbusAddress=43,.CANAddress=1111,.name={"Torque angle"},.shortName={"Trq ang"},.description={"Difference between stator and rotor angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.1f,.minValue=-180.0f,.maxValue=180.0f,.defaultValue=0.0f},
		{.number=44,.ModbusAddress=44,.CANAddress=1111,.name={"U current"},.shortName={"I U"},.description={"Actual U phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=45,.ModbusAddress=45,.CANAddress=1111,.name={"V current"},.shortName={"I V"},.description={"Actual V phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=46,.ModbusAddress=46,.CANAddress=1111,.name={"W current"},.shortName={"I W"},.description={"Actual W phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplierMB=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},

};

uint16_t parameter_list_size=sizeof(parameter_list)/sizeof(parameter_t);
uint16_t monitor_list_size=sizeof(monitor_list)/sizeof(parameter_t);

/**
 * @brief  Get parameter value from internal registers.
 * @param  parameter to read
 * @param pointer to variable holding raw 4 bytes of data, cast is needed to appropriate type for given parameter
 * @retval HAL_OK if data copied fine, HAL ERROR if parameter number is not implemented and nothing was copied
 */

HAL_StatusTypeDef parameter_read(parameter_t * par , uint32_t * ptrToReturnValue){
	switch(par->number){
	case 1:{int16_t value =(int16_t)inverter.filtered_rotor_speed;memcpy(ptrToReturnValue,&value,2);break;}
	case 2:{float value=inverter.I_RMS;memcpy(ptrToReturnValue,&value,4);break;}
	case 3:{float value=(inverter.I_d_filtered/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	case 4:{float value=(inverter.I_q_filtered/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	case 5:{float value=(inverter.I_q_filtered/parameter_set.motor_nominal_current)*100.0f;memcpy(ptrToReturnValue,&value,4);break;}
	case 6:{int16_t value = (int16_t)((int32_t)axis.actual_position & 0xFFFF);memcpy(ptrToReturnValue,&value,2);break;} //split int32 to int16
	case 7:{int16_t value = (int16_t)(((int32_t)axis.actual_position>>16) & 0xFFFF);memcpy(ptrToReturnValue,&value,2);break;}
	case 8:{int16_t value = (int16_t)(axis.target_position & 0xFFFF);memcpy(ptrToReturnValue,&value,2);break;}
	case 9:{int16_t value = (int16_t)((axis.target_position>>16) & 0xFFFF);memcpy(ptrToReturnValue,&value,2);break;}
	case 10:{float value=(inverter.output_voltage/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	case 11:{float value=(inverter.DCbus_voltage);memcpy(ptrToReturnValue,&value,4);break;}

	case 20:{uint16_t value=inverter.error;memcpy(ptrToReturnValue,&value,2);break;}
	case 30:{ //status word creation
		uint16_t value=0x00;
		if(inverter.state==not_ready_to_switch_on || inverter.state==switch_on_disabled || inverter.state==faulted){bitclear(value,0);}else{bitset(value,0);} //bit0 ready to switch on
		if(inverter.state==switched_on || inverter.state==operation_enabled || inverter.state==quickstop_active){bitset(value,1);}else{bitclear(value,1);} //bit1 switched on
		if(inverter.state==operation_enabled || inverter.state==quickstop_active){bitset(value,2);}else{bitclear(value,2);} //bit2 operation enabled
		if(inverter.error!=0x00){bitset(value,3);}else{bitclear(value,3);}
		if(inverter.softstart_finished==1){bitset(value,4);}else{bitclear(value,4);}
		if(inverter.state==quickstop_active){bitset(value,5);}else{bitclear(value,5);}
		if(inverter.state==switch_on_disabled){bitset(value,6);}else{bitclear(value,6);}
		//@TODO: implement warning bit
		bitset(value,9);//remote bit always set @TODO: clear bit if commanded from menu/io terminal
		//@TODO: implement target reached bit for other modes
		if(inverter.control_mode==foc_speed){
			if(inverter.speed_setpoint-inverter.filtered_rotor_speed>=(inverter.speed_setpoint*0.2f) || inverter.speed_setpoint-inverter.filtered_rotor_speed<=(inverter.speed_setpoint*-0.2f)){bitclear(value,10);}else{bitset(value,10);}
		}
		//@TODO:internal limit active bit
		//@TODO:operation specific bits in statusword
		memcpy(ptrToReturnValue,&value,2);
		break;
	}
	case 31:{{uint16_t value=0x0FFF;memcpy(ptrToReturnValue,&value,2);break;}}
	case 32:{int16_t value=inverter.speed_setpoint;memcpy(ptrToReturnValue,&value,2);break;}
	case 33:{float value=(inverter.torque_current_setpoint/parameter_set.motor_nominal_current)*100.0f;memcpy(ptrToReturnValue,&value,4);break;}
	case 34:{float value=(inverter.field_current_setpoint/parameter_set.motor_nominal_current)*100.0f;memcpy(ptrToReturnValue,&value,4);break;}
	case 35:{float value=(inverter.stator_field_speed/(_2_PI/inverter.control_loop_freq));memcpy(ptrToReturnValue,&value,4);break;}
	case 36:{float value=(inverter.stator_field_speed/(_2_PI/inverter.control_loop_freq));memcpy(ptrToReturnValue,&value,4);break;} //@TODO:implement target open loop speed
	case 37:{float value=(inverter.output_voltage/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}

	case 40:{uint16_t value=(inverter.encoder_raw_position);memcpy(ptrToReturnValue,&value,2);break;}
	case 41:{float value=(inverter.stator_electric_angle*(180.0f/_PI));memcpy(ptrToReturnValue,&value,4);break;}
	case 42:{float value=(inverter.rotor_electric_angle*(180.0f/_PI));memcpy(ptrToReturnValue,&value,4);break;}
	case 43:{float value=(inverter.torque_angle*(180.0f/_PI));memcpy(ptrToReturnValue,&value,4);break;}
	case 44:{float value=(inverter.I_U);memcpy(ptrToReturnValue,&value,4);break;}
	case 45:{float value=(inverter.I_V);memcpy(ptrToReturnValue,&value,4);break;}
	case 46:{float value=(inverter.I_W);memcpy(ptrToReturnValue,&value,4);break;}

	case 100:{int16_t value=(int16_t)inverter.control_mode;memcpy(ptrToReturnValue,&value,2);break;}
	case 101:{uint16_t value=0;memcpy(ptrToReturnValue,&value,2);break;}

	case 120:{uint16_t value=parameter_set.position_factor_numerator;memcpy(ptrToReturnValue,&value,2);break;}
	case 121:{uint16_t value=parameter_set.position_factor_denominator;memcpy(ptrToReturnValue,&value,2);break;}

	case 200:{uint16_t value=(uint16_t)parameter_set.motor_feedback_type;memcpy(ptrToReturnValue,&value,2);break;}
	case 201:{float value=parameter_set.encoder_electric_angle_correction*(180.0f/_PI);memcpy(ptrToReturnValue,&value,4);break;}
	case 202:{uint16_t value=parameter_set.encoder_resolution/2;memcpy(ptrToReturnValue,&value,2);break;}
	case 203:{uint16_t value=parameter_set.encoder_polarity;memcpy(ptrToReturnValue,&value,2);break;}

	case 210:{uint16_t value=parameter_set.motor_pole_pairs;memcpy(ptrToReturnValue,&value,2);break;}
	case 211:{float value=(parameter_set.motor_nominal_current/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	case 212:{float value=(parameter_set.motor_max_current/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	case 213:{float value=(parameter_set.motor_voltage/_SQRT2);memcpy(ptrToReturnValue,&value,4);break;}
	case 214:{float value=(parameter_set.motor_base_frequency);memcpy(ptrToReturnValue,&value,4);break;}
	case 215:{float value=(parameter_set.motor_nominal_torque);memcpy(ptrToReturnValue,&value,4);break;}
	case 216:{float value=(parameter_set.motor_nominal_speed);memcpy(ptrToReturnValue,&value,4);break;}
	case 217:{float value=(parameter_set.motor_max_speed);memcpy(ptrToReturnValue,&value,4);break;}
	case 218:{float value=(parameter_set.motor_rs*2.0f);memcpy(ptrToReturnValue,&value,4);break;}
	case 219:{float value=(parameter_set.motor_ls*2000.0f);memcpy(ptrToReturnValue,&value,4);break;}
	case 220:{float value=(parameter_set.motor_K);memcpy(ptrToReturnValue,&value,4);break;}
	case 221:{float value=(parameter_set.motor_inertia*100.0f);memcpy(ptrToReturnValue,&value,4);break;}

	case 300:{float value=(parameter_set.acceleration_ramp_s);memcpy(ptrToReturnValue,&value,4);break;}
	case 301:{float value=(parameter_set.deceleration_ramp_s);memcpy(ptrToReturnValue,&value,4);break;}
	case 302:{float value=(parameter_set.speed_limit_positive);memcpy(ptrToReturnValue,&value,4);break;}
	case 303:{float value=(parameter_set.speed_limit_negative);memcpy(ptrToReturnValue,&value,4);break;}

	case 310:{float value=(parameter_set.position_controller_proportional_gain);memcpy(ptrToReturnValue,&value,4);break;}
	case 311:{float value=(parameter_set.position_controller_integral_gain);memcpy(ptrToReturnValue,&value,4);break;}

	case 320:{float value=(parameter_set.speed_controller_proportional_gain);memcpy(ptrToReturnValue,&value,4);break;}
	case 321:{float value=(parameter_set.speed_controller_integral_gain);memcpy(ptrToReturnValue,&value,4);break;}

	case 323:{float value=(parameter_set.speed_filter_ts*1000.0f);memcpy(ptrToReturnValue,&value,4);break;}

	case 330:{float value=(parameter_set.torque_current_ctrl_proportional_gain);memcpy(ptrToReturnValue,&value,4);break;}
	case 331:{float value=(parameter_set.torque_current_ctrl_integral_gain);memcpy(ptrToReturnValue,&value,4);break;} //@TODO: change pi controllers to series type to get integration time instead of i gain

	case 900:
	default:{return HAL_ERROR;break;}
	}
	return HAL_OK;
}
/**
 * @brief  Write value from HMI/fieldbus to internal variables
 * @param par pointer to parameter to write
 * @param pValue data to write, variable type depends of parameter->type, needs to be copied as raw bytes to variable of appropriate type
 * @retval HAL_BUSY is write protected or not possible to write in actual conditions, HAL_ERROR if parameter does not exist
 */
HAL_StatusTypeDef parameter_write(parameter_t * par, uint32_t * pValue){
	HAL_StatusTypeDef status = HAL_OK;
	switch(par->WriteAllowed){
	case 0:{status = HAL_ERROR;break;} //read only
	case 2:{
		if(inverter.state==operation_enabled){status = HAL_BUSY;break;}	}//if write allowed on stop only break out from switch, otherwise fall through to write allowed
	case 1:{ //write allowed
		switch(par->number){
		case 8:{
			int16_t value=0;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			axis.target_position=axis.target_position & 0xFFFF0000; //clear lower 16bit
			axis.target_position=axis.target_position | ((int32_t)value & 0x0000FFFF);
			break;
		}
		case 9:{
			int16_t value=0;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			axis.target_position=axis.target_position & 0x0000FFFF; //clear lower 16bit
			axis.target_position=axis.target_position | ((((int32_t)value)<<16) & 0xFFFF0000);
			break;
		}
		case 31:{ //control register
			uint16_t value = (uint16_t)*pValue;
			if(!bitcheck(value,1)){//enable voltage bit =0 - disable inverter, transition 7,9,10,12
				inverter_disable();
			}
			if(bitcheck(value,1) && bitcheck(value,2) && inverter.state==switch_on_disabled){//enable voltage bit =1 - ready to switch on, transition 2
				if(!inverter.softstart_finished)inverter_error_trip(undervoltage);
				if(inverter.softstart_finished)inverter.state=ready_to_switch_on;
			}
			if(bitcheck(value,0) && bitcheck(value,2)&& inverter.state==ready_to_switch_on){ //switch on bit =1 - switched on state, transition 3
				inverter.state=switched_on;
			}
			if(!bitcheck(value,0) &&  bitcheck(value,2) && (inverter.state==switched_on ||inverter.state==operation_enabled || inverter.state==quickstop_active)){ //switch on bit = 0
				inverter.state=ready_to_switch_on;
				inverter_disable();
			}
			if(!bitcheck(value,2)){ //bit 2 quickstop has to be high in other transitions, if low go to quickstop when operation enabled, otherwise switch on disabled
				if(inverter.state==operation_enabled){inverter_disable();inverter.state=quickstop_active;}
				if(inverter.state==ready_to_switch_on || inverter.state==switched_on);
			}
			if(bitcheck(value,3) && bitcheck(value,2)&& bitcheck(value,1) && bitcheck(value,0)&& (inverter.state==switched_on||inverter.state==quickstop_active)){ //transition 4
				inverter_enable();
			}
			if(!bitcheck(value,3) && bitcheck(value,2)&& bitcheck(value,1) && bitcheck(value,0) && inverter.state == operation_enabled){
				inverter_disable();inverter.state=switched_on;
			}
			if(bitcheck(value,7) && inverter.state==faulted){
				inverter_error_reset();
			}
			break;
		}
		case 20:
		{
			if(*pValue==0){inverter_error_reset();}
			break;
		}
		case 32:{ //target speed
			int16_t value =0;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			if(inverter.control_mode!=foc_speed){status = HAL_BUSY;break;}else{
				inverter.speed_setpoint=value;break;
			}break;
		}
		case 33: //target torque
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			if(inverter.control_mode!=foc_torque){status = HAL_BUSY;break;}else{
				inverter.torque_current_setpoint=(value/100.0f)*parameter_set.motor_nominal_current;break;
			}break;
		}
		case 34: //target field
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			if(inverter.control_mode<-1 ){status = HAL_BUSY;break;}else{
				inverter.field_current_setpoint=(value/100.0f)*parameter_set.motor_nominal_current;break;
			}break;
		}
		case 36: //stator field speed target
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			if(inverter.control_mode==open_loop_current || inverter.control_mode==u_f || inverter.control_mode==manual){
				inverter.stator_field_speed = (value*(_2_PI/inverter.control_loop_freq));
			}break;
		}
		case 37: //stator volt target
				{
					float value = 0.0f;
					if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
					if(inverter.control_mode==manual){
						inverter.output_voltage = (value*_SQRT2);
					}break;
				}
		case 100: //control mode
		{
			int16_t value=0;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			if(value==5 || value==0 || inverter.state==operation_enabled){status=HAL_BUSY;break;}else{
				parameter_set.control_mode=(operation_mode_t)value;
			}
			break;
		}
		case 101: //write to eeprom
		{
			uint16_t value=0;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			if(value==1){
				if(save_parameter_set_to_eeprom()!=HAL_OK){
					inverter_error_trip(eeprom_error);
				}
			}
			break;}
		case 120: //position factor numerator
		{
			uint16_t value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.position_factor_numerator=(uint16_t)value;
			break;}
		case 121: //position factor denominator
		{
			uint16_t value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.position_factor_denominator=(uint16_t)value;
			break;}

		case 200: //feedback type
		{
			uint16_t value=0;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_feedback_type=(motor_feedback_type_t)value;
			break;}
		case 201: //feedback angle correction
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.encoder_electric_angle_correction=value/(180.0f/_PI);
			break;}
		case 202: //enc resolution
		{
			uint16_t value = 0;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.encoder_resolution=value*2;
			break;}
		case 203: //encoder polarity
		{
			uint16_t value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.encoder_polarity=value;
			break;}
		case 210: //motor pole pairs
		{
			uint16_t value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_pole_pairs=(uint8_t)value;
			break;}
		case 211: //nominal current
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_nominal_current=value*_SQRT2;
			break;}
		case 212: //max current
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_max_current=value*_SQRT2;
			break;}
		case 213: //nominal voltage
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_voltage=value*_SQRT2;
			break;}
		case 214: //base frequency
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_base_frequency=value;
			break;}
		case 215: //nominal torque
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_nominal_torque=value;
			break;}
		case 216: //nominal speed
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_nominal_speed=value;
			break;}
		case 217: //max speed
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_max_speed=value;
			break;}
		case 218: //Rs
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_rs=value/2.0f;
			break;}
		case 219: //Ls
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_ls=(value/1000.0f)/2.0f;
			break;}
		case 220: //Kv
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_K=value;
			break;}
		case 221: //inertia
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.motor_inertia=value/100.0f;
			break;}
		case 300: //acc
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.acceleration_ramp_s=value;
			break;}
		case 301: //dec
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.deceleration_ramp_s=value;
			break;}
		case 302: //speed limit+
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.speed_limit_positive=value;
			break;}
		case 303: //speed limit-
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.speed_limit_negative=value;
			break;}
		case 310: //position p
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.position_controller_proportional_gain=value;
			break;}
		case 311: //position i
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.position_controller_integral_gain=value;
			break;}
		case 320: //speed p
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.speed_controller_proportional_gain=value;
			break;}
		case 321: //speed i
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.speed_controller_integral_gain=value;
			break;}
		case 323: //speed filter tau
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.speed_filter_ts=value/1000.0f;
			break;}
		case 330: //current p
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.torque_current_ctrl_proportional_gain=value;
			parameter_set.field_current_ctrl_proportional_gain=value;
			break;}
		case 331: //current i
		{
			float value = 0.0f;
			if(prepare_received_data(par, pValue, &value)!=HAL_OK){break;}
			parameter_set.torque_current_ctrl_integral_gain=value;
			parameter_set.field_current_ctrl_integral_gain=value;
			break;}
		default:{status=HAL_ERROR;break;}
		}
	}
	}
	inverter.constant_values_update_needed=1;
	return status;
}

HAL_StatusTypeDef limiter(parameter_t * par, void * pValue, void * pReturnValue){
	HAL_StatusTypeDef status=HAL_OK;
	switch(par->type){
	case pBOOL16: case pUINT16:{
		uint16_t value = *(uint16_t*)pValue; //cast pointer to appropriate type for given parameter
		if(value > par->maxValue){status=HAL_ERROR;break;}
		if(value < par-> minValue){status=HAL_ERROR;break;}
		*(uint16_t*)pReturnValue=value;
		break;
	}
	case pINT16:{
		int16_t value = *(int16_t*)pValue; //cast pointer to appropriate type for given parameter
		if(value > par->maxValue){status=HAL_ERROR;break;}
		if(value < par-> minValue){status=HAL_ERROR;break;}
		*(int16_t*)pReturnValue=value;
		break;
	}
	case pFLOAT:{
		float value = *(float*)pValue; //cast pointer to appropriate type for given parameter
		if(value > par->maxValue){status=HAL_ERROR;break;}
		if(value < par-> minValue){status=HAL_ERROR;break;}
		*(float*)pReturnValue=value;
		break;
	}
	}
	return status;
}

HAL_StatusTypeDef prepare_received_data(parameter_t * par, uint32_t * pReceivedData, void * pReturnValue){
	HAL_StatusTypeDef status=HAL_OK;
	switch(par->type){
	case pBOOL16: case pUINT16:{
		memcpy(pReturnValue,pReceivedData,2);
		status=limiter(par,pReturnValue,pReturnValue);
		break;
	}
	case pINT16:{
		memcpy(pReturnValue,pReceivedData,2);
		status=limiter(par,pReturnValue,pReturnValue);
		break;
	}
	case pFLOAT:{
		memcpy(pReturnValue,pReceivedData,4);
		status=limiter(par,pReturnValue,pReturnValue);
		break;
	}
	}
	return status;
}

uint16_t par_get_index_CAN(uint16_t CAN_address){
	for(uint16_t i=0;i<parameter_list_size;i++){
		if(parameter_list[i].CANAddress==CAN_address)return i;
	}
	return 0xFFFF;
}
