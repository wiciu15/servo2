/*
 * comm_modbus.c
 *
 *  Created on: Feb 27, 2022
 *      Author: Wiktor
 */

#include "main.h"
#include "comm_modbus.h"
#include "modbus.h"
#include <string.h>
#include "inverter.h"
#include "usbd_cdc_if.h"

modbus_instance_t * ptrModbusUSBinstance;

uint16_t modbus_protocol_read(uint32_t la){

	uint8_t local_address=la-40000;
	uint16_t response=0xFFFF;
	switch (local_address){
	case 2: response = inverter.error;break;
	case 3: response = inverter.state;break;
	case 5: response = inverter.control_mode;break;
	case 6:{if(inverter.control_mode==manual || inverter.control_mode==open_loop_current){response = (int16_t)(inverter.stator_field_speed/(_2_PI/MOTOR_CTRL_LOOP_FREQ))*10;}if(inverter.control_mode==foc){response = inverter.speed_setpoint;} break;}
	case 7:{if(inverter.control_mode==manual || inverter.control_mode==u_f){response = (uint16_t)(inverter.output_voltage*10.0f);}if(inverter.control_mode==open_loop_current || inverter.control_mode == foc){response = (int16_t)((inverter.torque_current_setpoint/parameter_set.motor_nominal_current)*1000.0f);}break;}
	case 8:{if(inverter.control_mode==open_loop_current || inverter.control_mode==foc){response = (int16_t)((inverter.field_current_setpoint/parameter_set.motor_nominal_current)*1000.0f);}break;}
	case 10: response = (int16_t)(inverter.I_RMS *100.0f);break;
	case 11: response = (uint16_t)(inverter.stator_electric_angle*(180.0f/_PI)*100.0f);break;
	case 12: response = (int16_t)(inverter.torque_angle*(180.0f/_PI));break;
	case 13: response = (int16_t)(inverter.filtered_rotor_speed);break;
	case 14: response = (uint16_t)(inverter.DCbus_voltage*10.0f);break;
	case 15: response = (int16_t)(inverter.I_d_filtered*100.0f);break;
	case 16: response = (int16_t)(inverter.I_q_filtered*100.0f);break;
	case 17: response = inverter.encoder_raw_position;break;
	case 18: response = inverter.output_voltage*10.0f;break;
	case 19: response = (int16_t)(inverter.IGBT_temp*10.0f);break;

	case 20: response = parameter_set.motor_feedback_type;break;
	case 21: response = (int16_t)(parameter_set.encoder_electric_angle_correction*(180.0f/_PI));break;
	case 22: response = parameter_set.encoder_resolution;break;
	//case 23: if(parameter_set.motor_feedback_type==mitsubishi_encoder){response = mitsubishi_encoder_data.excessive_acceleration_error_count;}if(parameter_set.motor_feedback_type==tamagawa_encoder){response = tamagawa_encoder_data.excessive_acceleration_error_count;}break;
	case 31: response = parameter_set.motor_pole_pairs;break;
	case 32: response = (parameter_set.motor_nominal_current/_SQRT2)*100.0f;break;
	case 33: response = (parameter_set.motor_max_current/parameter_set.motor_nominal_current)*100.0f;break;
	case 34: response = (parameter_set.motor_max_current/_SQRT2)*100.0f;break;
	case 35: response = (parameter_set.motor_max_voltage/_SQRT2);break;
	case 36: response = parameter_set.motor_nominal_torque*100.0f;break;
	case 37: response = parameter_set.motor_nominal_speed;break;
	case 38: response = parameter_set.motor_max_speed;break;
	case 39: response = parameter_set.motor_rs*1000.0f;break;
	case 40: response = parameter_set.motor_ls*1000000.0f;break;
	case 41: response = parameter_set.motor_K*100000.0f;break;
	case 42: response = (parameter_set.motor_base_frequency/(_2_PI/MOTOR_CTRL_LOOP_FREQ))*100;break;
	case 50: response = parameter_set.torque_current_ctrl_proportional_gain*10.0f;break;
	case 51: response = parameter_set.torque_current_ctrl_integral_gain;break;
	case 52: response = parameter_set.field_current_ctrl_proportional_gain*10.0f;break;
	case 53: response = parameter_set.field_current_ctrl_integral_gain;break;
	case 54: response = parameter_set.current_filter_ts*10000.0f;break;
	case 55: response = parameter_set.speed_controller_proportional_gain*1000.0f;break;
	case 56: response = parameter_set.speed_controller_integral_gain*1000.0f;break;
	case 57: response = parameter_set.speed_filter_ts*10000.0f;break;

	}
	return (uint16_t)response;
}

uint16_t modbus_protocol_write(uint32_t la, uint16_t value)
{
	uint8_t local_address=la-40000;
	switch (local_address){

	case 2://error register
		{if(value==0){inverter.error = no_error;inverter.state=stop;}break; //acknowledge error
		break;}
	case 3: //control register
		{switch(value){
			case 0:
				inverter_disable();inverter.state=stop;break;
			case 1:
				inverter.state=run;inverter_enable();break;
			case 3:
				inverter_error_trip(external_comm);
			default:
				inverter_disable();break;
			}
	break;}

	case 5: //operation mode register
	{if(value<=3){inverter.control_mode=value;}
	break;
	}

	case 6: //speed setpoint in rpm
	{int16_t received_speed=value;
	if(inverter.control_mode==manual || inverter.control_mode==u_f || inverter.control_mode==open_loop_current){
		if((received_speed)<=5000 && (received_speed)>=(-5000) ){inverter.stator_field_speed = ((float)received_speed*(_2_PI/MOTOR_CTRL_LOOP_FREQ))/10.0f;}
	}
	if(inverter.control_mode==foc){
		if((received_speed)<=5000 && (received_speed)>=(-5000) ){inverter.speed_setpoint = received_speed;}
	}
	break;}

	case 7: //set output voltage in manual/torque in foc
	{if(inverter.control_mode==manual){
		if(value<=4000 && value>=0){inverter.output_voltage = ((float)value/10.0f);}
		}
	if(inverter.control_mode==open_loop_current || inverter.control_mode==foc ){
		int16_t received_torque_setpoint = (int16_t)value;
		if(received_torque_setpoint>=-3000 && received_torque_setpoint<=3000){
			if(inverter.speed_setpoint==0.0f){
				inverter.torque_current_setpoint=(received_torque_setpoint/1000.0f)*parameter_set.motor_nominal_current;
			}
		}
	}
	break;
	}

	case 8:
	{
		if(inverter.control_mode==open_loop_current || inverter.control_mode==foc){
			int16_t received_field_setpoint = value;
			if(received_field_setpoint>=-1000 && received_field_setpoint<=1000){
				inverter.field_current_setpoint=(received_field_setpoint/1000.0f)*parameter_set.motor_nominal_current;
			}
		}
		break;
	}

	//feedback type
	case 20:
	{
		uint16_t received_feedback_type = value;
		if(received_feedback_type<=3 && inverter.state!=run){
			parameter_set.motor_feedback_type=value;}
		break;}
	//Encoder angle correction
	case 21:
	{
		int16_t received_value = value;
		if(received_value>=-180 && received_value<=180){
			parameter_set.encoder_electric_angle_correction=received_value/(180.0f/_PI);
		}
		break;}
	//Encoder resolution
	case 22:
	{
		uint16_t received_value = value;
		if(received_value>=250 && received_value<=65536){
			parameter_set.encoder_resolution=received_value;
		}
		break;}
	//Motor pole pairs
	case 31:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=12){
			parameter_set.motor_pole_pairs=received_value;
		}
		break;}
	//Motor nominal current
	case 32:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=1600){
			parameter_set.motor_nominal_current=(received_value/100.0f)*_SQRT2;
		}
		break;}
	//Motor overload factor
	case 33:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=400){
			parameter_set.motor_max_current=parameter_set.motor_nominal_current*(received_value/100.0f);
		}
		break;}
	//Motor max voltage
	case 35:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=500){
			parameter_set.motor_max_voltage=((float)received_value)*_SQRT2;
		}
		break;}
	//Motor nominal torque
	case 36:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=4000){
			parameter_set.motor_nominal_torque=(float)received_value/100.0f;
		}
		break;}
	//Motor nominal speed
	case 37:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=6000){
			parameter_set.motor_nominal_speed=received_value;
		}
		break;}
	case 38:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=8000){
			parameter_set.motor_max_speed=received_value;
		}
		break;}
	//Motor Rs
	case 39:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=60000){
			parameter_set.motor_rs=(float)received_value/1000.0f;
		}
		break;}
	//Motor Ls
	case 40:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=60000){
			parameter_set.motor_ls=(float)received_value/1000000.0f;
		}
		break;}
	//Motor Kv
	case 41:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=60000){
			parameter_set.motor_K=(float)received_value/100000.0f;
		}
		break;}
	//motor base frequency for u/f control
	case 42:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=60000){
			parameter_set.motor_base_frequency=((float)received_value/100.0f)*(_2_PI/MOTOR_CTRL_LOOP_FREQ);
		}
		break;}
	case 50:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=900){
			parameter_set.torque_current_ctrl_proportional_gain=(float)received_value/10.0f;
		}
		break;}
	case 51:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=20000){
			parameter_set.torque_current_ctrl_integral_gain=(float)received_value;
		}
		break;}
	case 52:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=900){
			parameter_set.field_current_ctrl_proportional_gain=(float)received_value/10.0f;
		}
		break;}

	case 53:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=20000){
			parameter_set.field_current_ctrl_integral_gain=(float)received_value;
		}
		break;}
	//current filter
	case 54:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=10000){
			parameter_set.current_filter_ts=(float)received_value/10000.0f;
		}
		break;}
	case 55:
		{
			uint16_t received_value = value;
			if(received_value>=1 && received_value<=20000){
				parameter_set.speed_controller_proportional_gain=(float)received_value/1000.0f;
			}
			break;}
	case 56:
		{
			uint16_t received_value = value;
			if(received_value>=1 && received_value<=20000){
				parameter_set.speed_controller_integral_gain=(float)received_value/1000.0f;
			}
			break;}
	//speed filter
	case 57:
	{
		uint16_t received_value = value;
		if(received_value>=1 && received_value<=10000){
			parameter_set.speed_filter_ts=(float)received_value/10000.0f;
		}
		break;}

	default:
		//if not handled inside switch, then read-only parameter
		break;
	}
return value;
}

int mbus_send(const mbus_t context,const uint8_t* data, const uint16_t size){
	UNUSED(context);
	//HAL_GPIO_WritePin(MODBUS_DE_GPIO_Port,MODBUS_DE_Pin, 1);
	//if(HAL_UART_Transmit_DMA( &huart1, (uint8_t*) data,size)==HAL_OK){
	if(CDC_Transmit_FS((uint8_t*) data, size)==HAL_OK){
		return MBUS_OK;
	}else{return MBUS_ERROR;}
}

void ModbusUSB_init(modbus_instance_t* mbus_instance){
	/* Device slave address */
	mbus_instance->mb_config.devaddr = 0x01;

	/* Just ptr on any external object, you can get it by context */
	mbus_instance->mb_config.device = (void*) 0;

	uint8_t * pmodbusSendBuffer;
	pmodbusSendBuffer=mbus_instance->modbusSendBuffer;
	mbus_instance->mb_config.sendbuf = pmodbusSendBuffer;
	mbus_instance->mb_config.sendbuf_sz = sizeof(mbus_instance->modbusSendBuffer);

	uint8_t * pmodbusRecvBuffer;
	pmodbusRecvBuffer=mbus_instance->modbusReceiveBuffer;
	mbus_instance->mb_config.recvbuf = pmodbusRecvBuffer;
	mbus_instance->mb_config.recvbuf_sz = sizeof(mbus_instance->modbusReceiveBuffer);

	/* This that function for sending some data (use sendbuf for buf) */
	mbus_instance->mb_config.send = &mbus_send;

	Modbus_Conf_t * pconf;
	pconf=&mbus_instance->mb_config;
	//User Read callback function ( read by logical address)
	pconf->read = modbus_protocol_read;

	//Write callback function
	pconf->write = modbus_protocol_write;

	//Open modbus contex
	mbus_instance->modbus = mbus_open(pconf);

	//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART_RX_buf, sizeof(UART_RX_buf));
}

void modbus_process_new_data_to_fifo(modbus_instance_t* mbus_instance, uint8_t * buffer,uint32_t Size){
		mbus_flush(mbus_instance->modbus);
		/* start the DMA again */
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) UART_RX_buf, sizeof(UART_RX_buf));
		//__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

		mbus_instance->fifo_oldpos = mbus_instance->fifo_newpos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (mbus_instance->fifo_oldpos+Size > sizeof(mbus_instance->RX_FIFO)-1)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = sizeof(mbus_instance->RX_FIFO)-mbus_instance->fifo_oldpos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)mbus_instance->RX_FIFO+mbus_instance->fifo_oldpos, buffer, datatocopy);  // copy data in that remaining space

			mbus_instance->fifo_oldpos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)mbus_instance->RX_FIFO, (uint8_t*)buffer+datatocopy, (Size-datatocopy));  // copy the remaining data
			mbus_instance->fifo_newpos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)mbus_instance->RX_FIFO+mbus_instance->fifo_oldpos, buffer, Size);
			mbus_instance->fifo_newpos = Size+mbus_instance->fifo_oldpos;
		}




}

void process_modbus_command(modbus_instance_t* mbus_instance){
	while(mbus_instance->fifo_read_pos!=mbus_instance->fifo_newpos){
		mbus_poll(mbus_instance->modbus, mbus_instance->RX_FIFO[mbus_instance->fifo_read_pos] );
		mbus_instance->fifo_read_pos++;
		if(mbus_instance->fifo_read_pos>=sizeof(mbus_instance->RX_FIFO)){
			mbus_instance->fifo_read_pos=0;
		}
	}

}
