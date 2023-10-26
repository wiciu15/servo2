/*
 * parameter_list.c
 *
 *  Created on: Aug 2, 2023
 *      Author: Wiktor
 */

#include "parameter_list.h"
#include <string.h>

const parameter_t parameter_list[]={
		{.number=1,.ModbusAddress=1,.CANAddress=1111,.name={"Actual speed"},.shortName={"Act spd"},.description={"Actual motor speed"},.WriteAllowed=0,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-10000.0f,.maxValue=10000.0f,.defaultValue=0.0f},
		{.number=2,.ModbusAddress=2,.CANAddress=1111,.name={"Apparent current"},.shortName={"I mot"},.description={"Actual motor RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=3,.ModbusAddress=3,.CANAddress=1111,.name={"Magnetizing current"},.shortName={"I mag"},.description={"Magnetizing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=4,.ModbusAddress=4,.CANAddress=1111,.name={"Torque current"},.shortName={"I trq"},.description={"Torque-producing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=5,.ModbusAddress=5,.CANAddress=1111,.name={"Motor torque"},.shortName={"Mot trq"},.description={"Actaul motor torque in %"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=6,.ModbusAddress=6,.CANAddress=1111,.name={"Actual position L"},.shortName={"Act posL"},.description={"Actual rotor position low word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=7,.ModbusAddress=7,.CANAddress=1111,.name={"Actual position H"},.shortName={"Act posH"},.description={"Actual axis position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-32767.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=8,.ModbusAddress=8,.CANAddress=1111,.name={"Target position L"},.shortName={"Tht posL"},.description={"Actual rotor position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=9,.ModbusAddress=9,.CANAddress=1111,.name={"Target position H"},.shortName={"Tgt posH"},.description={"Target axis position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-32767.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=10,.ModbusAddress=10,.CANAddress=1111,.name={"Motor voltage"},.shortName={"U mot"},.description={"Actual motor RMS voltage"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=11,.ModbusAddress=11,.CANAddress=1111,.name={"DC bus voltage"},.shortName={"DC bus"},.description={"Actual voltage in intermediate circuit"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=12,.ModbusAddress=12,.CANAddress=1111,.name={"Motor thermal load"},.shortName={"Mot I2t"},.description={"Motor thermal calculated from output current"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=20,.ModbusAddress=20,.CANAddress=1111,.name={"Actual error"},.shortName={"Act Err"},.description={"Actual error number"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=21,.ModbusAddress=21,.CANAddress=1111,.name={"Error history 1"},.shortName={"Err1"},.description={"Error history 1(latest)"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=22,.ModbusAddress=22,.CANAddress=1111,.name={"Error history 2"},.shortName={"Err2"},.description={"Error history 2"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=23,.ModbusAddress=23,.CANAddress=1111,.name={"Error history 3"},.shortName={"Err3"},.description={"Error history 3"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=24,.ModbusAddress=24,.CANAddress=1111,.name={"Error history 4"},.shortName={"Err4"},.description={"Error history 4"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=25,.ModbusAddress=25,.CANAddress=1111,.name={"Error history 5"},.shortName={"Err5"},.description={"Error history 5"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=26,.ModbusAddress=26,.CANAddress=1111,.name={"Error history 6"},.shortName={"Err6"},.description={"Error history 6"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=27,.ModbusAddress=27,.CANAddress=1111,.name={"Error history 7"},.shortName={"Err7"},.description={"Error history 7"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=28,.ModbusAddress=28,.CANAddress=1111,.name={"Error history 8"},.shortName={"Err8"},.description={"Error history 8"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=29,.ModbusAddress=29,.CANAddress=1111,.name={"Error history 9"},.shortName={"Err9"},.description={"Error history 9"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=30,.ModbusAddress=30,.CANAddress=1111,.name={"Status register"},.shortName={"Sts reg"},.description={"Status register"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=31,.ModbusAddress=31,.CANAddress=1111,.name={"Control register"},.shortName={"Ctl reg"},.description={"Control register"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=32,.ModbusAddress=32,.CANAddress=1111,.name={"Target speed"},.shortName={"Tgt spd"},.description={"Target speed"},.WriteAllowed=1,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-30000.0f,.maxValue=30000.0f,.defaultValue=0.0f},
		{.number=33,.ModbusAddress=33,.CANAddress=1111,.name={"Target torque"},.shortName={"Tgt trq"},.description={"Target torque"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=34,.ModbusAddress=34,.CANAddress=1111,.name={"Field torque"},.shortName={"Fld trq"},.description={"Target field"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=35,.ModbusAddress=35,.CANAddress=1111,.name={"Open loop freq"},.shortName={"OL freq"},.description={"Actual output frequency in uf/open loop mode"},.WriteAllowed=0,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},
		{.number=36,.ModbusAddress=36,.CANAddress=1111,.name={"Target open loop freq"},.shortName={"Freq tgt"},.description={"Output frequency target in uf/open loop mode"},.WriteAllowed=1,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},

		{.number=40,.ModbusAddress=40,.CANAddress=1111,.name={"Encoder pulse"},.shortName={"Enc puls"},.description={"One turn absolute encoder pulse"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=41,.ModbusAddress=41,.CANAddress=1111,.name={"Stator Electric Angle"},.shortName={"Fld ang"},.description={"Actual stator field electric angle"},.WriteAllowed=1,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=359.9f,.defaultValue=0.0f},
		{.number=42,.ModbusAddress=42,.CANAddress=1111,.name={"Rotor Electric Angle"},.shortName={"Rot ang"},.description={"Actual rotor field electric angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=360.0f,.defaultValue=0.0f},
		{.number=43,.ModbusAddress=43,.CANAddress=1111,.name={"Torque angle"},.shortName={"Trq ang"},.description={"Difference between stator and rotor angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=-180.0f,.maxValue=180.0f,.defaultValue=0.0f},
		{.number=44,.ModbusAddress=44,.CANAddress=1111,.name={"U current"},.shortName={"I U"},.description={"Actual U phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=45,.ModbusAddress=45,.CANAddress=1111,.name={"V current"},.shortName={"I V"},.description={"Actual V phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=46,.ModbusAddress=46,.CANAddress=1111,.name={"W current"},.shortName={"I W"},.description={"Actual W phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},

		{.number=100,.ModbusAddress=100,.CANAddress=1111,.name={"Motor control mode"},.shortName={"Ctl mode"},.description={"Motion control mode"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-5.0f,.maxValue=7.0f,.defaultValue=1.0f},

};

const parameter_t monitor_list[]={ //parameters shown in monitor menu
		{.number=1,.ModbusAddress=1,.CANAddress=1111,.name={"Actual speed"},.shortName={"Act spd"},.description={"Actual motor speed"},.WriteAllowed=0,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-10000.0f,.maxValue=10000.0f,.defaultValue=0.0f},
		{.number=2,.ModbusAddress=2,.CANAddress=1111,.name={"Apparent current"},.shortName={"I mot"},.description={"Actual motor RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=3,.ModbusAddress=3,.CANAddress=1111,.name={"Magnetizing current"},.shortName={"I mag"},.description={"Magnetizing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=4,.ModbusAddress=4,.CANAddress=1111,.name={"Torque current"},.shortName={"I trq"},.description={"Torque-producing RMS current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-32000.0f,.maxValue=32000.0f,.defaultValue=0.0f},
		{.number=5,.ModbusAddress=5,.CANAddress=1111,.name={"Motor torque"},.shortName={"Mot trq"},.description={"Actaul motor torque in %"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-1000.0f,.maxValue=1000.0f,.defaultValue=0.0f},
		{.number=6,.ModbusAddress=6,.CANAddress=1111,.name={"Actual position L"},.shortName={"Act posL"},.description={"Actual axis position low word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=7,.ModbusAddress=7,.CANAddress=1111,.name={"Actual position H"},.shortName={"Act posH"},.description={"Actual axis position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-32767.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=8,.ModbusAddress=8,.CANAddress=1111,.name={"Target position L"},.shortName={"Tgt posL"},.description={"Target axis position low word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=9,.ModbusAddress=9,.CANAddress=1111,.name={"Target position H"},.shortName={"Tgt posH"},.description={"Target axis position high word"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-32767.0f,.maxValue=32767.0f,.defaultValue=0.0f},
		{.number=10,.ModbusAddress=10,.CANAddress=1111,.name={"Motor voltage"},.shortName={"U mot"},.description={"Actual motor RMS voltage"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=11,.ModbusAddress=11,.CANAddress=1111,.name={"DC bus voltage"},.shortName={"DC bus"},.description={"Actual voltage in intermediate circuit"},.WriteAllowed=0,.precision=1,.unit="V",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=12,.ModbusAddress=12,.CANAddress=1111,.name={"Motor thermal load"},.shortName={"Mot I2t"},.description={"Motor thermal utilization calculated from current"},.WriteAllowed=0,.precision=1,.unit="%",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=20,.ModbusAddress=20,.CANAddress=1111,.name={"Actual error"},.shortName={"Act Err"},.description={"Actual error number"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=21,.ModbusAddress=21,.CANAddress=1111,.name={"Error history 1"},.shortName={"Err1"},.description={"Error history 1(latest)"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=22,.ModbusAddress=22,.CANAddress=1111,.name={"Error history 2"},.shortName={"Err2"},.description={"Error history 2"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=23,.ModbusAddress=23,.CANAddress=1111,.name={"Error history 3"},.shortName={"Err3"},.description={"Error history 3"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=24,.ModbusAddress=24,.CANAddress=1111,.name={"Error history 4"},.shortName={"Err4"},.description={"Error history 4"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=25,.ModbusAddress=25,.CANAddress=1111,.name={"Error history 5"},.shortName={"Err5"},.description={"Error history 5"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=26,.ModbusAddress=26,.CANAddress=1111,.name={"Error history 6"},.shortName={"Err6"},.description={"Error history 6"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=27,.ModbusAddress=27,.CANAddress=1111,.name={"Error history 7"},.shortName={"Err7"},.description={"Error history 7"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=28,.ModbusAddress=28,.CANAddress=1111,.name={"Error history 8"},.shortName={"Err8"},.description={"Error history 8"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=29,.ModbusAddress=29,.CANAddress=1111,.name={"Error history 9"},.shortName={"Err9"},.description={"Error history 9"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},

		{.number=30,.ModbusAddress=30,.CANAddress=1111,.name={"Status register"},.shortName={"Sts reg"},.description={"Status register"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=31,.ModbusAddress=31,.CANAddress=1111,.name={"Control register"},.shortName={"Ctl reg"},.description={"Control register"},.WriteAllowed=1,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pBOOL16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=32,.ModbusAddress=32,.CANAddress=1111,.name={"Target speed"},.shortName={"Tgt spd"},.description={"Target speed"},.WriteAllowed=1,.precision=0,.unit="RPM",.ModbusDataType=mbINT16,.type=pINT16,.multiplier=1.0f,.minValue=-30000.0f,.maxValue=30000.0f,.defaultValue=0.0f},
		{.number=33,.ModbusAddress=33,.CANAddress=1111,.name={"Target torque"},.shortName={"Tgt trq"},.description={"Target torque"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=34,.ModbusAddress=34,.CANAddress=1111,.name={"Target field"},.shortName={"Fld trq"},.description={"Target field"},.WriteAllowed=1,.precision=2,.unit="%",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=35,.ModbusAddress=35,.CANAddress=1111,.name={"Open loop freq"},.shortName={"OL freq"},.description={"Actual output frequency in uf/open loop mode"},.WriteAllowed=0,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},
		{.number=36,.ModbusAddress=36,.CANAddress=1111,.name={"Target open loop freq"},.shortName={"Freq tgt"},.description={"Output frequency target in uf/open loop mode"},.WriteAllowed=1,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},
		{.number=37,.ModbusAddress=37,.CANAddress=1111,.name={"Target voltage"},.shortName={"Tgt U"},.description={"Output voltage target in manual mode"},.WriteAllowed=1,.precision=1,.unit="Hz",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=-650.0f,.maxValue=650.0f,.defaultValue=0.0f},


		{.number=40,.ModbusAddress=40,.CANAddress=1111,.name={"Encoder pulse"},.shortName={"Enc puls"},.description={"One turn absolute encoder pulse"},.WriteAllowed=0,.precision=0,.unit="",.ModbusDataType=mbUINT16,.type=pUINT16,.multiplier=1.0f,.minValue=0.0f,.maxValue=65535.0f,.defaultValue=0.0f},
		{.number=41,.ModbusAddress=41,.CANAddress=1111,.name={"Stator Electric Angle"},.shortName={"Fld ang"},.description={"Actual stator field electric angle"},.WriteAllowed=1,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=359.9f,.defaultValue=0.0f},
		{.number=42,.ModbusAddress=42,.CANAddress=1111,.name={"Rotor Electric Angle"},.shortName={"Rot ang"},.description={"Actual rotor field electric angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbUINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=0.0f,.maxValue=360.0f,.defaultValue=0.0f},
		{.number=43,.ModbusAddress=43,.CANAddress=1111,.name={"Torque angle"},.shortName={"Trq ang"},.description={"Difference between stator and rotor angle"},.WriteAllowed=0,.precision=1,.unit="deg",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.1f,.minValue=-180.0f,.maxValue=180.0f,.defaultValue=0.0f},
		{.number=44,.ModbusAddress=44,.CANAddress=1111,.name={"U current"},.shortName={"I U"},.description={"Actual U phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=45,.ModbusAddress=45,.CANAddress=1111,.name={"V current"},.shortName={"I V"},.description={"Actual V phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},
		{.number=46,.ModbusAddress=46,.CANAddress=1111,.name={"W current"},.shortName={"I W"},.description={"Actual W phase current"},.WriteAllowed=0,.precision=2,.unit="A",.ModbusDataType=mbINT16,.type=pFLOAT,.multiplier=0.01f,.minValue=-300.0f,.maxValue=300.0f,.defaultValue=0.0f},

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
	case 36:{float value=(inverter.stator_field_speed/(_2_PI/inverter.control_loop_freq));memcpy(ptrToReturnValue,&value,4);break;} //implement target open loop speed

	case 40:{uint16_t value=(inverter.encoder_raw_position);memcpy(ptrToReturnValue,&value,2);break;}
	case 41:{float value=(inverter.stator_electric_angle*(180.0f/_PI));memcpy(ptrToReturnValue,&value,4);break;}
	case 42:{float value=(inverter.rotor_electric_angle*(180.0f/_PI));memcpy(ptrToReturnValue,&value,4);break;}
	case 43:{float value=(inverter.torque_angle*(180.0f/_PI));memcpy(ptrToReturnValue,&value,4);break;}
	case 44:{float value=(inverter.I_U);memcpy(ptrToReturnValue,&value,4);break;}
	case 45:{float value=(inverter.I_V);memcpy(ptrToReturnValue,&value,4);break;}
	case 46:{float value=(inverter.I_W);memcpy(ptrToReturnValue,&value,4);break;}



	default:{return HAL_ERROR;break;}
	}
	return HAL_OK;
}
HAL_StatusTypeDef parameter_write(parameter_t * par, uint32_t * pValue){
	HAL_StatusTypeDef status = HAL_OK;
	switch(par->WriteAllowed){
	case 0:{status = HAL_ERROR;break;} //read only
	case 2:{
		if(inverter.state==operation_enabled){status = HAL_BUSY;break;}	}//if write allowed on stop only break out from switch, otherwise fall through to write allowed
	case 1:{ //write allowed
		switch(par->number){
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
				inverter_enable();inverter.state=operation_enabled;
			}
			if(!bitcheck(value,3) && bitcheck(value,2)&& bitcheck(value,1) && bitcheck(value,0) && inverter.state == operation_enabled){
				inverter_disable();inverter.state=switched_on;
			}
			if(bitcheck(value,7) && inverter.state==faulted){
				inverter_error_reset();
			}
			break;
		}
		case 32:{ //target speed
			int16_t value = (int16_t)*pValue;
			if(value>8000||value<-8000||inverter.control_mode!=foc_speed){status = HAL_BUSY;break;}else{
				inverter.speed_setpoint=value;break;
			}
		}
		case 33: //target torque
		{
			float value = 0.0f;
			memcpy(&value,pValue,4);
			if(value>300.0f||value<-300.0f||inverter.control_mode!=foc_torque){status = HAL_BUSY;break;}else{
				inverter.torque_current_setpoint=(value/100.0f)*parameter_set.motor_nominal_current;break;
			}
		}

		default:{status=HAL_ERROR;break;}
	}
	}
}
return status;
}
