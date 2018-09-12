#include <iostream>
#include <cstring>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "TCPServerBase.hpp"

using namespace std;
using namespace boost::asio;

TCPServerBase::TCPServerBase(unsigned int Port) :
        m_acceptor(m_io, endpoint_type(ip::tcp::v4(), Port)) {
    accept();
};

TCPServerBase::~TCPServerBase() {
};

void TCPServerBase::run() {
    m_io.run();
}

void TCPServerBase::stop() {
    m_io.stop();
}

void TCPServerBase::accept() {
    sock_ptr sock(new socket_type(m_io));

    m_acceptor.async_accept(*sock,
                            bind(&this_type::accept_handler, this, boost::asio::placeholders::error, sock));
}

void TCPServerBase::accept_handler(const boost::system::error_code &ec, sock_ptr sock) {
    if (ec) {
        cout << "accept_handler error." << boost::system::system_error(ec).what() << endl;
        return;
    }

    cout << "receive message from " << sock->remote_endpoint().address() << endl;

    boost::array<char, 1024> buf;
    boost::system::error_code error;

    size_t len = sock->read_some(boost::asio::buffer(buf), error);

    if (error == boost::asio::error::eof) {

    } else if (error) {
        throw boost::system::system_error(error); // Some other error.
    } else {
        const string bufString(buf.data(), len);
        // cout << bufString << endl;

        string response = onHandleTCP(bufString);
        if (!response.empty()) {
            try {
                cout << "response: \"" << response << "\"" << endl;
                boost::system::error_code ignored_error;
                sock->write_some(boost::asio::buffer(response), ignored_error);
            } catch (std::exception &e) {
                std::cerr << e.what() << std::endl;
                // throw e; // comment this line to aviod throwing
            }
        }
    }

    accept();
}
