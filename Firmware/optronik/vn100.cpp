// 
// 
// 

//_____ I N C L U D E S ________________________________________________________
#include "vn100.h"

//_____ D E F I N I T I O N S __________________________________________________

//_____ F U N C T I O N S ______________________________________________________
VN100::VN100(HardwareSerial &uart, unsigned long baud) :_uart(&uart)
{
	_baud = baud;
	_uart->begin(_baud);
	_header = "$VNYMR";
}

VN100::VN100(HardwareSerial &uart, unsigned long baud, String header) :_uart(&uart)
{
	_baud = baud;
	_uart->begin(_baud);
	_header = header;
}


bool VN100::read()
{
	char a, c;
	String tem, val[3];
	byte awal, akhir, i;
	long tem1, tem2;
	bool ret = 0;

	while (_uart->available()) {
		c = _uart->read();
		//    Serial.write(c);
		if (c == _header.charAt(0)) {
			in = "";
			inCompleted = false;
		}
		else if (c == '*') {
			inCompleted = true;
			break;
		}
		in += c;
	}

	if (inCompleted) {
		//    Serial.println(in);
		if (in.indexOf(_header) == 0) {
			akhir = in.indexOf(',');
			for (i = 0; i<3; i++) {
				awal = akhir + 1;
				akhir = in.indexOf(',', awal);
				tem = in.substring(awal, akhir);
				val[i] = tem;
			}
			for (i = 0; i<3; i++) {
				a = val[i].charAt(0);
				awal = val[i].indexOf('.');
				tem = val[i].substring(1, awal);
				tem1 = long(tem.toInt()) * 1000;
				tem = val[i].substring(awal + 1);
				tem2 = tem.toInt();
				ypr[i] = tem1 + tem2;
				if (a == '-') {
					ypr[i] *= -1;
				}
			}
			ret = 1;
		}
	}

	return ret;
}

void VN100::getYpr(long * val)
{
	val[0] = ypr[0];
	val[1] = ypr[1];
	val[2] = ypr[2];
}

