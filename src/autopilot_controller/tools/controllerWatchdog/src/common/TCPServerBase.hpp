#ifndef _TCPSERVER_HPP__
#define _TCPSERVER_HPP__

#include <boost/asio.hpp>

class TCPServerBase {
    typedef TCPServerBase this_type;
    typedef boost::asio::ip::tcp::acceptor acceptor_type;
    typedef boost::asio::ip::tcp::endpoint endpoint_type;
    typedef boost::asio::ip::tcp::socket socket_type;
    typedef std::shared_ptr<socket_type> sock_ptr;

    boost::asio::io_service m_io;
    acceptor_type m_acceptor;

public:
    TCPServerBase(unsigned int Port);

    virtual ~TCPServerBase();

    void run();

    void stop();

    void accept();

    void accept_handler(const boost::system::error_code &ec, sock_ptr sock);

    virtual std::string onHandleTCP(const std::string tcpMessage) = 0;

};

#endif /*_TCPSERVER_HPP__*/
