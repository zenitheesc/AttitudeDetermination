//
// WebSocketServer.cpp
//
// This sample demonstrates the WebSocket class.
//
// Copyright (c) 2012, Applied Informatics Software Engineering GmbH.
// and Contributors.
//
// SPDX-License-Identifier:	BSL-1.0
//
#include "Poco/Net/WebSocket.h"
#include "Poco/Net/HTTPRequestHandler.h"
#include "Poco/Net/HTTPRequestHandlerFactory.h"
#include "Poco/Net/HTTPServer.h"
#include "Poco/Net/HTTPServerParams.h"
#include "Poco/Net/HTTPServerRequest.h"
#include "Poco/Net/HTTPServerResponse.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/Util/ServerApplication.h"
#include "alglin/alglin.hpp"
#include "attdet/attdet.h"
#include <algorithm>
#include <iostream>

#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define REGEX_STR                                                       \
	"([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[" \
	"\\d]+),"                                                           \
	"([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[" \
	"\\d]+),"                                                           \
	"([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[\\d]+),([\\-]?[\\d]+\\.[" \
	"\\d]+)."                                                           \
	"\\D"


using Poco::Net::WebSocket;
using Poco::Net::HTTPRequestHandler;
using Poco::Net::HTTPRequestHandlerFactory;
using Poco::Net::HTTPServer;
using Poco::Net::HTTPServerRequest;
using Poco::Net::HTTPServerResponse;
using Poco::Net::HTTPServerParams;
using Poco::Net::ServerSocket;


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
		newtio.c_cflag =
		  static_cast<int>(baudrate) | CRTSCTS | CS8 | CLOCAL | CREAD;
		newtio.c_iflag = IGNPAR | ICRNL;

		newtio.c_oflag = 0;
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


/** WebSockets começam como HTTP normal */
struct PageRequestHandler : public HTTPRequestHandler {
	void handleRequest(HTTPServerRequest &, HTTPServerResponse &res) override {
		res.setChunkedTransferEncoding(true);
		res.setContentType("text/html");
		auto &ostr = res.send();
		ostr << "<h1>WebSocket Only</h1>";
	}
};


/** Contem o loop do socket */
struct WebSocketRequestHandler : public HTTPRequestHandler {
	void handleRequest(
	  HTTPServerRequest &request, HTTPServerResponse &response) override {
		WebSocket ws(request, response);
		std::cout << "WebSocket connection established.\n";
		int flags{ 129 };
		int n{};

		const std::regex data_regex(REGEX_STR);
		static std::smatch s_data;
        
		const Vec3 m_ref({ -4., -18., -20. });
		const Vec3 a_ref({ 0.16, -0.4, -9.4 });
		using namespace attdet;
		auto mag_sensor = Sensor({ 0., 1., 0. }, alglin::normalize(m_ref), .40);
		auto acc_sensor = Sensor({ 0., 1., 0. }, alglin::normalize(a_ref), .60);

		SerialRead<255> serial("/dev/ttyUSB0", baud::b115200);
		do {

			while (true) {
				auto line = std::string{ serial.readline() };

				if (std::regex_match(line, s_data, data_regex)) {
					std::vector<float> data{};
					std::for_each(s_data.begin() + 1,
					  s_data.end(),
					  [&data](decltype(*s_data.begin()) x) {
						  float f = std::atof(x.str().c_str());
						  data.push_back(f);
					  });

					acc_sensor.measure =
					  alglin::normalize(Vec3({ data[0], data[1], data[2] }));
					mag_sensor.measure =
					  alglin::normalize(Vec3({ data[6], data[7], data[8] }));

					auto q = quest({ acc_sensor, mag_sensor });
					std::stringstream ss;
					ss << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3]
					   << '\n';

					// std::cout << ss.str();
					ws.sendFrame(ss.str().c_str(), ss.str().size());
				}
			}
		} while (n > 0
				 && (flags & WebSocket::FRAME_OP_BITMASK)
					  != WebSocket::FRAME_OP_CLOSE);
		std::cout << "WebSocket connection closed.\n";
	}
};

/** Primeiro a receber o request e checa se é pra usar WebSockets */
struct RequestHandlerFactory : public HTTPRequestHandlerFactory {
	HTTPRequestHandler *createRequestHandler(
	  const HTTPServerRequest &req) override {
		std::cout << "Request " << req.clientAddress().toString() << '\n';
		if (req.find("Upgrade") != req.end()
			&& Poco::icompare(req["Upgrade"], "websocket") == 0) {
			return new WebSocketRequestHandler;
		} else {
			return new PageRequestHandler;
		}
	}
};


/** (http://localhost:9980/) */
struct WebSocketServer : public Poco::Util::ServerApplication {
	int main(const std::vector<std::string> &) {
		ServerSocket socket_server((unsigned short)9980);
		HTTPServer http_server(
		  new RequestHandlerFactory, socket_server, new HTTPServerParams);
		http_server.start();
		waitForTerminationRequest();
		http_server.stop();

		return 0;
	}
};

int main(int argc, char **argv) {
	WebSocketServer app;
	return app.run(argc, argv);
}
