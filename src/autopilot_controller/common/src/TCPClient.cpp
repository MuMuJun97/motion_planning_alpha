#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <unistd.h>
#include "TCPClient.hpp"

void writeTCPMsg2Sock(const std::string &ip, const unsigned int port, const std::string &msg) {
    std::cout << "send \"" << msg << "\" to " << ip << ":" << port << std::endl;
    boost::array<char, 256> buf;

    try {
        boost::asio::io_service io_service;
        boost::asio::ip::tcp::endpoint end_point(boost::asio::ip::address::from_string(ip), port);

        boost::asio::ip::tcp::socket socket(io_service);
        socket.connect(end_point);

        boost::system::error_code ignored_error;
        socket.write_some(boost::asio::buffer(msg), ignored_error);

//		size_t len = socket.read_some(boost::asio::buffer(buf), ignored_error);
//		if (len > 0) {
//			std::cout << "Received: ";
//			std::cout.write(buf.data(), len);
//		}
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        throw e; // comment this line to aviod throwing
    }

}
