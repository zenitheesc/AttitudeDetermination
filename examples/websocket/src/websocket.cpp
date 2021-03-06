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
#include <algorithm>
#include <iostream>


using Poco::Net::WebSocket;
using Poco::Net::HTTPRequestHandler;
using Poco::Net::HTTPRequestHandlerFactory;
using Poco::Net::HTTPServer;
using Poco::Net::HTTPServerRequest;
using Poco::Net::HTTPServerResponse;
using Poco::Net::HTTPServerParams;
using Poco::Net::ServerSocket;

/** WebSockets começam como HTTP normal */
struct PageRequestHandler : public HTTPRequestHandler {
	void handleRequest(HTTPServerRequest &r, HTTPServerResponse &response) {
		response.setChunkedTransferEncoding(true);
		response.setContentType("text/html");
		auto &ostr = response.send();
		ostr << "<h1>WebSocket Only</h1>";
	}
};

/** Contem o loop do socket */
struct WebSocketRequestHandler : public HTTPRequestHandler {
	void handleRequest(
	  HTTPServerRequest &request, HTTPServerResponse &response) {
		WebSocket ws(request, response);
		std::cout << "WebSocket connection established.\n";
		char buffer[1024] = {0};
        int flags{ 129 };
		int n{};
		do {

			ws.sendFrame(buffer, n, flags);

		} while (n > 0
				 && (flags & WebSocket::FRAME_OP_BITMASK)
					  != WebSocket::FRAME_OP_CLOSE);
		std::cout << "WebSocket connection closed.\n";
	}
};

/** Primeiro a receber o request e checa se é pra usar WebSockets */
struct RequestHandlerFactory : public HTTPRequestHandlerFactory {
	HTTPRequestHandler *createRequestHandler(const HTTPServerRequest &request) {
		std::cout << "Request " << request.clientAddress().toString() << '\n';
		if (request.find("Upgrade") != request.end()
			&& Poco::icompare(request["Upgrade"], "websocket") == 0) {
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
