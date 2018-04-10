// 
// 
// 

#include "libVisca.h"

LibVisca::LibVisca(HardwareSerial & uart, unsigned long baud):_uart(&uart)
{
	_baud = baud;
	//_uart->begin(baud);
}


/********************************/
/*      PRIVATE FUNCTIONS       */
/********************************/

void LibVisca::_append_byte(VISCAPacket_t * packet, uint8_t data)
{
	packet->bytes[packet->length] = data;
	(packet->length)++;
}

void LibVisca::_init_packet(VISCAPacket_t * packet)
{
	// we start writing at byte 1, the first byte will be filled by the
	// packet sending function. This function will also append a terminator.
	packet->length = 1;
}

uint8_t LibVisca::_get_reply(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	//first messages: ----------------------
	if (_get_packet(iface) != VISCA_SUCCESS) return VISCA_FAILURE;
	iface->type = iface->ibuf[1] & 0xF0;

	//skip ack messages
	while (iface->type == VISCA_RESPONSE_ACK) {
		if (_get_packet(iface) != VISCA_SUCCESS) return VISCA_FAILURE;
		iface->type = iface->ibuf[1] & 0xF0;
	}

	switch (iface->type)
	{
	case VISCA_RESPONSE_CLEAR:
		return VISCA_SUCCESS;
		break;
	case VISCA_RESPONSE_ADDRESS:
		return VISCA_SUCCESS;
		break;
	case VISCA_RESPONSE_COMPLETED:
		return VISCA_SUCCESS;
		break;
	case VISCA_RESPONSE_ERROR:
		return VISCA_SUCCESS;
		break;
	}
	return VISCA_FAILURE;
}

uint8_t LibVisca::_send_packet_with_reply(VISCAInterface_t * iface, VISCACamera_t * camera, VISCAPacket_t * packet)
{
	if (_rx_clear() != VISCA_SUCCESS) return VISCA_FAILURE;
	if (_send_packet(iface, camera, packet) != VISCA_SUCCESS) return VISCA_FAILURE;
	if (_get_reply(iface, camera) != VISCA_SUCCESS) return VISCA_FAILURE;
	
	
	return VISCA_SUCCESS;
}

uint8_t LibVisca::_write_packet_data(VISCAInterface_t * iface, VISCACamera_t * camera, VISCAPacket_t * packet)
{
	for (int i = 0; i < packet->length; i++)
		_uart->write(packet->bytes[i]);

	return VISCA_SUCCESS;
}

uint8_t LibVisca::_send_packet(VISCAInterface_t * iface, VISCACamera_t * camera, VISCAPacket_t * packet)
{
	//check data
	if ((iface->address > 7) || (camera->address > 7) || (iface->broadcast)) {
#ifdef VISCA_DEBUG
		Serial.println(F("_send_packet: bad header parms"));
#endif // VISCA_DEBUG
		return VISCA_FAILURE;
	}
	
	//build header
	packet->bytes[0] = 0x80 | (iface->address << 4);
	if (iface->broadcast > 0) {
		packet->bytes[0] |= iface->broadcast << 3;
		packet->bytes[0] &= 0xF8;
	}
	else packet->bytes[0] |= camera->address;

	// append footer
	_append_byte(packet, VISCA_TERMINATOR);

	return _write_packet_data(iface, camera, packet);
}

uint8_t LibVisca::_get_packet(VISCAInterface_t * iface)
{
	uint32_t timer = 0;
	int pos = 0;
	byte b;

	timer = millis() + VISCA_SERIAL_WAIT;

	while (millis() < timer) 
	{
		if (_uart->available()) 
		{
			iface->ibuf[pos] = (uint8_t)_uart->read();
			if (iface->ibuf[pos] == VISCA_TERMINATOR) break;
			else 
			{
				if (++pos >= VISCA_INPUT_BUFFER_SIZE) 
				{
#ifdef VISCA_DEBUG
					Serial.println(F("_get_packet: overflow"));
#endif // VISCA_DEBUG
					return VISCA_FAILURE;
				}
			}

		}
	}
	iface->bytes = pos + 1;

	if (millis() >= timer) 
	{
#ifdef VISCA_DEBUG
		Serial.println(F("_get_packet: timeout"));
#endif // VISCA_DEBUG
		return VISCA_FAILURE;
	}
	else return VISCA_SUCCESS;
}

uint8_t LibVisca::_rx_clear()
{
	uint32_t _timer = millis() + VISCA_SERIAL_WAIT;

	while (_uart->available()) {
		if (millis() > _timer) return VISCA_FAILURE;
		else _uart->read();
	}

	return VISCA_SUCCESS;
}

void LibVisca::_tx_flush()
{
	_uart->flush();
}


/*******************************/
/*      PUBLIC FUNCTIONS       */
/*******************************/

void LibVisca::open_serial()
{
	_uart->begin(_baud);
}

void LibVisca::close_serial()
{
	_uart->end();
}

uint8_t LibVisca::set_address(VISCAInterface_t * iface, uint8_t * camera_num)
{
	VISCAPacket_t packet;
	int backup;
	VISCACamera_t camera;
	uint8_t a;

	camera.address = 0;
	backup = iface->broadcast;
	_init_packet(&packet);
	_append_byte(&packet, 0x30);
	_append_byte(&packet, 0x01);

	iface->broadcast = 1;
	a = _send_packet(iface, &camera, &packet);
	iface->broadcast = backup;
	if ( a!= VISCA_SUCCESS) return VISCA_FAILURE;
	if (_get_reply(iface, &camera) != VISCA_SUCCESS) return VISCA_FAILURE;
	else
	{
		//We parse the message from the camera here
		//We expect to receive 4*camera_num bytes,
		//every packet should be 88 30 0x FF, x being
		//the camera id+1. The number of cams will thus be
		//ibuf[bytes-2]-1  

		if (iface->bytes & 0x3 != 0)	/*check multiple of 4*/
			return VISCA_FAILURE;
		else
		{
			*camera_num = iface->ibuf[iface->bytes - 2] - 1;
			if ((*camera_num == 0) || (*camera_num > 7))
				return VISCA_FAILURE;
			else
				return VISCA_SUCCESS;
		}
	}
}

uint8_t LibVisca::clear(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, 0x01);
	_append_byte(&packet, 0x00);
	_append_byte(&packet, 0x01);

	if (_send_packet(iface, camera, &packet) != VISCA_SUCCESS)
		return VISCA_FAILURE;
	else
		if (_get_reply(iface, camera) != VISCA_SUCCESS)
			return VISCA_FAILURE;
		else
			return VISCA_SUCCESS;
}

uint8_t LibVisca::get_camera_info(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;
	packet.bytes[0] = 0x80 | camera->address;
	packet.bytes[1] = 0x09;
	packet.bytes[2] = 0x00;
	packet.bytes[3] = 0x02;
	packet.bytes[4] = VISCA_TERMINATOR;
	packet.length = 5;

	if (_write_packet_data(iface, camera, &packet) != VISCA_SUCCESS)
		return VISCA_FAILURE;
	else
		if (_get_reply(iface, camera) != VISCA_SUCCESS)
			return VISCA_FAILURE;

	if (iface->bytes != 10) /* we expect 10 bytes as answer */
		return VISCA_FAILURE;
	else
	{
		camera->vendor = (iface->ibuf[2] << 8) + iface->ibuf[3];
		camera->model = (iface->ibuf[4] << 8) + iface->ibuf[5];
		camera->rom_version = (iface->ibuf[6] << 8) + iface->ibuf[7];
		camera->socket_num = iface->ibuf[8];
		return VISCA_SUCCESS;
	}
}

/***********************************/
/*       SYSTEM  FUNCTIONS         */
/***********************************/

uint8_t LibVisca::set_power(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_POWER);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zoom_tele(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM);
	_append_byte(&packet, VISCA_ZOOM_TELE);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_irreceive_onoff(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
	_append_byte(&packet, VISCA_IRRECEIVE);
	_append_byte(&packet, VISCA_IRRECEIVE_ONOFF);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_irreceive_off(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
	_append_byte(&packet, VISCA_IRRECEIVE);
	_append_byte(&packet, VISCA_IRRECEIVE_OFF);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_irreceive_on(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
	_append_byte(&packet, VISCA_IRRECEIVE);
	_append_byte(&packet, VISCA_IRRECEIVE_ON);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_title(VISCAInterface_t * iface, VISCACamera_t * camera, VISCATitleData_t * title)
{
	VISCAPacket_t packet;
	int i, err = 0;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_TITLE_SET);
	_append_byte(&packet, VISCA_TITLE_SET_PART1);

	for (i = 0; i<10; i++)
		_append_byte(&packet, title->title[i]);

	err += _send_packet_with_reply(iface, camera, &packet);

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_TITLE_SET);
	_append_byte(&packet, VISCA_TITLE_SET_PART2);

	for (i = 0; i<10; i++)
		_append_byte(&packet, title->title[i + 10]);

	err += _send_packet_with_reply(iface, camera, &packet);

	return err;
}

uint8_t LibVisca::set_title_params(VISCAInterface_t * iface, VISCACamera_t * camera, VISCATitleData_t * title)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_TITLE_SET);
	_append_byte(&packet, VISCA_TITLE_SET_PARAMS);
	_append_byte(&packet, title->vposition);
	_append_byte(&packet, title->hposition);
	_append_byte(&packet, title->color);
	_append_byte(&packet, title->blink);
	_append_byte(&packet, 0);
	_append_byte(&packet, 0);
	_append_byte(&packet, 0);
	_append_byte(&packet, 0);
	_append_byte(&packet, 0);
	_append_byte(&packet, 0);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_title_clear(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_TITLE_DISPLAY);
	_append_byte(&packet, VISCA_TITLE_DISPLAY_CLEAR);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_title_display(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_TITLE_DISPLAY);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_time_display(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_TIME_DISPLAY);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_date_display(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DATE_DISPLAY);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_date_time(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DATE_TIME_SET);
	_append_byte(&packet, year / 10);
	_append_byte(&packet, year - 10 * (year / 10));
	_append_byte(&packet, month / 10);
	_append_byte(&packet, month - 10 * (month / 10));
	_append_byte(&packet, day / 10);
	_append_byte(&packet, day - 10 * (day / 10));
	_append_byte(&packet, hour / 10);
	_append_byte(&packet, hour - 10 * (hour / 10));
	_append_byte(&packet, minute / 10);
	_append_byte(&packet, minute - 10 * (minute / 10));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_display(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DISPLAY);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::memory_reset(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t channel)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_MEMORY);
	_append_byte(&packet, VISCA_MEMORY_RESET);
	_append_byte(&packet, channel);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::memory_recall(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t channel)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_MEMORY);
	_append_byte(&packet, VISCA_MEMORY_RECALL);
	_append_byte(&packet, channel);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::memory_set(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t channel)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_MEMORY);
	_append_byte(&packet, VISCA_MEMORY_SET);
	_append_byte(&packet, channel);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_stabilizer(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t mode)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_CAM_STABILIZER);
	_append_byte(&packet, mode);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_digital_effect_level(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t level)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DIGITAL_EFFECT_LEVEL);
	_append_byte(&packet, level);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_digital_effect(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t mode)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DIGITAL_EFFECT);
	_append_byte(&packet, mode);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_picture_effect(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t mode)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_PICTURE_EFFECT);
	_append_byte(&packet, mode);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_freeze(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FREEZE);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_mirror(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_MIRROR);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_wide_mode(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t mode)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_WIDE_MODE);
	_append_byte(&packet, mode);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_ir_led(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_IR_LED);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zero_lux_shot(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZERO_LUX);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_backlight_comp(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BACKLIGHT_COMP);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_slow_shutter_auto(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_SLOW_SHUTTER);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_auto_exp_mode(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t mode)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_AUTO_EXP);
	_append_byte(&packet, mode);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_exp_comp_power(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP_POWER);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP_POWER);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_exp_comp_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_exp_comp_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);

	return VISCA_SUCCESS;
}

uint8_t LibVisca::set_exp_comp_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_exp_comp_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_aperture_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_APERTURE_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_aperture_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_APERTURE);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_aperture_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_APERTURE);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_aperture_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_APERTURE);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bright_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BRIGHT_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bright_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BRIGHT);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bright_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BRIGHT);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bright_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BRIGHT);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_gain_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_GAIN_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_gain_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_GAIN);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_gain_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_GAIN);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_gain_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_GAIN);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_iris_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_IRIS_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_iris_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_IRIS);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_iris_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_IRIS);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_iris_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_IRIS);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_shutter_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_SHUTTER_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_shutter_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_SHUTTER);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_shutter_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_SHUTTER);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_shutter_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_SHUTTER);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bgain_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BGAIN_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bgain_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BGAIN);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bgain_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BGAIN);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_bgain_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BGAIN);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_rgain_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t value)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_RGAIN_VALUE);
	_append_byte(&packet, (value & 0xF000) >> 12);
	_append_byte(&packet, (value & 0x0F00) >> 8);
	_append_byte(&packet, (value & 0x00F0) >> 4);
	_append_byte(&packet, (value & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_rgain_reset(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_RGAIN);
	_append_byte(&packet, VISCA_RESET);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_rgain_down(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_RGAIN);
	_append_byte(&packet, VISCA_DOWN);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_rgain_up(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_RGAIN);
	_append_byte(&packet, VISCA_UP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_whitebal_one_push(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_WB_TRIGGER);
	_append_byte(&packet, VISCA_WB_ONE_PUSH_TRIG);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_whitebal_mode(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t mode)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_WB);
	_append_byte(&packet, mode);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_near_limit(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t limit)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_NEAR_LIMIT);
	_append_byte(&packet, (limit & 0xF000) >> 12);
	_append_byte(&packet, (limit & 0x0F00) >> 8);
	_append_byte(&packet, (limit & 0x00F0) >> 4);
	_append_byte(&packet, (limit & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_autosense_low(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE);
	_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE_LOW);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_autosense_high(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE);
	_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE_HIGH);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_infinity(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_ONE_PUSH);
	_append_byte(&packet, VISCA_FOCUS_ONE_PUSH_INF);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_one_push(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_ONE_PUSH);
	_append_byte(&packet, VISCA_FOCUS_ONE_PUSH_TRIG);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_auto(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_AUTO);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t focus)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_VALUE);
	_append_byte(&packet, (focus & 0xF000) >> 12);
	_append_byte(&packet, (focus & 0x0F00) >> 8);
	_append_byte(&packet, (focus & 0x00F0) >> 4);
	_append_byte(&packet, (focus & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_near_speed(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t speed)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS);
	_append_byte(&packet, VISCA_FOCUS_NEAR_SPEED | (speed & 0x7));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_far_speed(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t speed)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS);
	_append_byte(&packet, VISCA_FOCUS_FAR_SPEED | (speed & 0x7));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_stop(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS);
	_append_byte(&packet, VISCA_FOCUS_STOP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_near(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS);
	_append_byte(&packet, VISCA_FOCUS_NEAR);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_focus_far(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS);
	_append_byte(&packet, VISCA_FOCUS_FAR);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_dzoom_mode(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DZOOM_MODE);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_dzoom_limit(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t limit)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DZOOM_LIMIT);
	_append_byte(&packet, limit);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_dzoom(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DZOOM);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zoom_and_focus_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t zoom, uint32_t focus)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM_FOCUS_VALUE);
	_append_byte(&packet, (zoom & 0xF000) >> 12);
	_append_byte(&packet, (zoom & 0x0F00) >> 8);
	_append_byte(&packet, (zoom & 0x00F0) >> 4);
	_append_byte(&packet, (zoom & 0x000F));
	_append_byte(&packet, (focus & 0xF000) >> 12);
	_append_byte(&packet, (focus & 0x0F00) >> 8);
	_append_byte(&packet, (focus & 0x00F0) >> 4);
	_append_byte(&packet, (focus & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zoom_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t zoom)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM_VALUE);
	_append_byte(&packet, (zoom & 0xF000) >> 12);
	_append_byte(&packet, (zoom & 0x0F00) >> 8);
	_append_byte(&packet, (zoom & 0x00F0) >> 4);
	_append_byte(&packet, (zoom & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zoom_wide_speed(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t speed)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM);
	_append_byte(&packet, VISCA_ZOOM_WIDE_SPEED | (speed & 0x7));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zoom_tele_speed(VISCAInterface_t * iface, VISCACamera_t * camera, uint32_t speed)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM);
	_append_byte(&packet, VISCA_ZOOM_TELE_SPEED | (speed & 0x7));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zoom_stop(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM);
	_append_byte(&packet, VISCA_ZOOM_STOP);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_zoom_wide(VISCAInterface_t * iface, VISCACamera_t * camera)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM);
	_append_byte(&packet, VISCA_ZOOM_WIDE);

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_camera_id(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t id)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ID);
	_append_byte(&packet, (id & 0xF000) >> 12);
	_append_byte(&packet, (id & 0x0F00) >> 8);
	_append_byte(&packet, (id & 0x00F0) >> 4);
	_append_byte(&packet, (id & 0x000F));

	return _send_packet_with_reply(iface, camera, &packet);
}

uint8_t LibVisca::set_keylock(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t power)
{
	VISCAPacket_t packet;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_COMMAND);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_KEYLOCK);
	_append_byte(&packet, power);

	return _send_packet_with_reply(iface, camera, &packet);
}


/***********************************/
/*       INQUIRY FUNCTIONS         */
/***********************************/


uint8_t LibVisca::get_power(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_POWER);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_dzoom(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DZOOM);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else {
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_dzoom_limit(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DZOOM_LIMIT);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else {
		*value = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_zoom_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZOOM_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else {
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_focus_auto(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_AUTO);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_focus_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_focus_auto_sense(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * mode)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_AUTO_SENSE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*mode = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_focus_near_limit(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FOCUS_NEAR_LIMIT);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_whitebal_mode(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * mode)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_WB);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*mode = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_rgain_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_RGAIN_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_bgain_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BGAIN_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_auto_exp_mode(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * mode)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_AUTO_EXP);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*mode = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_slow_shutter_auto(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * mode)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_SLOW_SHUTTER);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*mode = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_shutter_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_SHUTTER_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_iris_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_IRIS_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_gain_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_GAIN_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_bright_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BRIGHT_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_exp_comp_power(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP_POWER);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_exp_comp_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_EXP_COMP_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_backlight_comp(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_BACKLIGHT_COMP);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_aperture_value(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_APERTURE_VALUE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_zero_lux_shot(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ZERO_LUX);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_ir_led(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_IR_LED);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_wide_mode(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * mode)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_WIDE_MODE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*mode = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_mirror(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_MIRROR);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_freeze(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_FREEZE);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_picture_effect(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * mode)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_PICTURE_EFFECT);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*mode = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_digital_effect(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * mode)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DIGITAL_EFFECT);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*mode = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_digital_effect_level(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * value)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DIGITAL_EFFECT_LEVEL);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*value = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_memory(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * channel)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_MEMORY);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*channel = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_display(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * power)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_DISPLAY);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*power = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_id(VISCAInterface_t * iface, VISCACamera_t * camera, uint16_t * id)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_CAMERA1);
	_append_byte(&packet, VISCA_ID);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*id = (iface->ibuf[2] << 12) + (iface->ibuf[3] << 8) + (iface->ibuf[4] << 4) + iface->ibuf[5];
		return VISCA_SUCCESS;
	}
}

uint8_t LibVisca::get_videosystem(VISCAInterface_t * iface, VISCACamera_t * camera, uint8_t * system)
{
	VISCAPacket_t packet;
	uint32_t err;

	_init_packet(&packet);
	_append_byte(&packet, VISCA_INQUIRY);
	_append_byte(&packet, VISCA_CATEGORY_PAN_TILTER);
	_append_byte(&packet, VISCA_PT_VIDEOSYSTEM_INQ);
	err = _send_packet_with_reply(iface, camera, &packet);
	if (err != VISCA_SUCCESS)
		return err;
	else
	{
		*system = iface->ibuf[2];
		return VISCA_SUCCESS;
	}
}
