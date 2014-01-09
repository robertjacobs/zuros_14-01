//id: 0403:6001

#include <C3mxl.h>
#include <LxFTDI.h>

int main(int argc, char** argv)
{
	C3mxlROS *motor = new C3mxl();
	LxSerial *serial_port = new LxFTDI();
	CDxlConfig *config = new CDxlConfig();

	serial_port->port_open("i:0x0403:6001", LxSerial::RS485_FTDI);
	serial_port->set_speed_int(921600);
	motor->setSerialPort(serial_port);

	motor->setConfig(config->setID(106));
	motor->init(false);
}
