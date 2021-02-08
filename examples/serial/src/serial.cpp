#include "serial.h"
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

enum class baud { b9600 = B9600, b115200 = B115200 };
template<int B = 255> struct SerialRead {
	int fd;
	char buffer[B];

	termios oldtio;
	SerialRead(const std::string &port, baud baudrate) {
		termios newtio;

		fd = open(port.c_str(), O_RDONLY | O_NOCTTY);
		if (fd < 0) {
			std::cerr << "Failed to open Serial Port\r\n";
			throw;
		}
		tcgetattr(fd, &oldtio); /* save current serial port settings */

		/*
		  BAUDRATE: Set bps rate. You could also use cfsetispeed and
		  cfsetospeed. CRTSCTS : output hardware flow control (only used if the
		  cable has all necessary lines. See sect. 7 of Serial-HOWTO) CS8     :
		  8n1 (8bit,no parity,1 stopbit) CLOCAL  : local connection, no modem
		  contol CREAD   : enable receiving characters
		*/
		newtio.c_cflag =
		  static_cast<int>(baudrate) | CRTSCTS | CS8 | CLOCAL | CREAD;

		/*
		  IGNPAR  : ignore bytes with parity errors
		  ICRNL   : map CR to NL (otherwise a CR input on the other computer
					will not terminate input)
		  otherwise make device raw (no other input processing)
		*/
		newtio.c_iflag = IGNPAR | ICRNL;

		/*
		 Raw output.
		*/
		newtio.c_oflag = 0;

		/*
		  ICANON  : enable canonical input
		  disable all echo functionality, and don't send signals to calling
		  program
		*/
		newtio.c_lflag = ICANON;

		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);// activate
	}

	char *readline() {
		int res = read(fd, this->buffer, B);
		this->buffer[res] = 0;
		return this->buffer;
	}

	~SerialRead() {
		tcsetattr(fd, TCSANOW, &oldtio);
		close(fd);
	}
};

int main() {
	using namespace attdet;
	const Matrix3 convertMagAxis(
	  { { 0., 1., 0. }, { 1., 0., 0. }, { 0., 0., -1. } });

	std::regex data_regex(
	  "([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),"
	  "([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),"
	  "([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+)."
	  // "([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+)"
	  "\\D");
	//!    ^
	std::smatch s_data;
	std::cout << std::fixed << std::setprecision(8);

	const Vec3 m_ref({ 1., -18.124316, -16.585069 });
	const Vec3 a_ref({ 0.1, -0.4, -9.5 });

	auto mag_sensor =
	  Sensor({ 0., 1., 0. }, alglin::normalize(convertMagAxis * m_ref), .40);
	auto acc_sensor = Sensor({ 0., 1., 0. }, alglin::normalize(a_ref), .60);

	SerialRead<255> serial("/dev/ttyUSB0", baud::b115200);
	while (true) {
		std::string line = serial.readline();
		if (std::regex_match(line, s_data, data_regex)) {
			std::vector<float> data{};
			std::for_each(s_data.begin() + 1, s_data.end(), [&data](auto x) {
				float f = std::atof(x.str().c_str());
				//  std::cout << f << '\t';
				data.push_back(f);
			});
			acc_sensor.measure =
			  alglin::normalize(Vec3({ data[0], data[1], data[2] }));
			mag_sensor.measure =
			  convertMagAxis
			  * alglin::normalize(Vec3({ data[6], data[7], data[8] }));

			// std::cout << acc_sensor.measure << mag_sensor.measure << '\n';
			auto q = quest({ acc_sensor, mag_sensor });
			std::cout << '\t' << acc_sensor.measure << "\t;"
					  << mag_sensor.measure << '\n';
			std::cout << '[' << q << "]\n";
			std::cout << '(' << Quat2Euler(q) << ")\n";
			// auto DCM = triad({acc_sensor, mag_sensor});
			// std::cout << DCM;
		}// else std::cout << "no data\n";
		// std::cout << '(' << line << ')';
	}
}
