#ifdef BUILD_COCO

#include "dwbecker.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "compat_string.h"
#include <sys/time.h>
#include <unistd.h> // write(), read(), close()
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR

#ifndef ESP_PLATFORM

#if !defined(_WIN32)
#  include <sys/ioctl.h>
#  include <netinet/tcp.h>
#endif

#ifndef MSG_NOSIGNAL
#  define MSG_NOSIGNAL 0
#  if defined(__APPLE__) || defined(__MACH__)
// MSG_NOSIGNAL does not exists on older macOS, use SO_NOSIGPIPE
#    define USE_SO_NOSIGPIPE
#  endif
#endif

#endif // !ESP_PLATFORM

#include "../../include/debug.h"

#include "fnSystem.h"
#include "fnWiFi.h"


#define DW_DEFAULT_BAUD         57600


// Constructor
BeckerPort::BeckerPort() :
    _host{0},
    _ip(IPADDR_NONE),
    _port(BECKER_DEFAULT_PORT),
    _baud(DW_DEFAULT_BAUD),     // not used by Becker
    _listening(true),
    _fd(-1),
    _listen_fd(-1),
    _state(&BeckerStopped::getInstance()),
    _errcount(0)
{}

BeckerPort::~BeckerPort()
{
    end();
}

void BeckerPort::begin(int baud)
{
    if (_state != &BeckerStopped::getInstance())
        end();

    _baud = baud;

    // listen or connect
    start_connection();
}

void BeckerPort::end()
{
    // close sockets
    if (_fd >= 0)
    {
        shutdown(_fd, 0);
        closesocket(_fd);
        _fd  = -1;
    }
    if (_listen_fd >= 0)
    {
        closesocket(_listen_fd);
        _listen_fd  = -1;
        Debug_printf("### BeckerPort stopped ###\n");
    }

    // wait a while, otherwise wifi may turn off too quickly (during shutdown)
    fnSystem.delay(50);

    setState(BeckerStopped::getInstance());
}

/* Returns number of bytes available in receive buffer or -1 on error
*/
int BeckerPort::available()
{
    // only in connected state
    if (_state != &BeckerConnected::getInstance())
        return 0;

    // check if socket is still connected
    if (!connected())
    {
        // connection was closed or it has an error
        suspend_on_disconnect();
        return 0;
    }

#if defined(_WIN32)
    unsigned long count;
    int res = ioctlsocket(_fd, FIONREAD, &count);
    res = res != 0 ? -1 : count;
#else
    int count;
    int res = ioctl(_fd, FIONREAD, &count);
    res = res < 0 ? -1 : count;
#endif
    return res;
}

/* Discards anything in the input buffer
*/
void BeckerPort::flush_input()
{
    // only in connected state
    if (_state != &BeckerConnected::getInstance())
        return;

    // waste all input data
    uint8_t rxbuf[256];
    int avail;
    while ((avail = available()) > 0)
    {
        recv(_fd, (char *)rxbuf, avail > sizeof(rxbuf) ? sizeof(rxbuf) : avail, 0);
    }
}

/* Clears input buffer and flushes out transmit buffer waiting at most
   waiting MAX_FLUSH_WAIT_TICKS until all sends are completed
*/
void BeckerPort::flush()
{
    // only in connected state
    if (_state != &BeckerConnected::getInstance())
        return;

    wait_sock_writable(250);
}

// specific to BeckerPort
void BeckerPort::set_host(const char *host, int port)
{
    if (host != nullptr)
        strlcpy(_host, host, sizeof(_host));
    else
        _host[0] = 0;

    _port = port;
}

const char* BeckerPort::get_host(int &port)
{
    port = _port;
    return _host;
}

void BeckerPort::start_connection()
{
    if (_listening)
        listen_for_connection();
    else
        make_connection();
}

void BeckerPort::listen_for_connection()
{

    // Wait for WiFi
    if (!fnWiFi.connected())
    {
        Debug_println("BeckerPort: No WiFi!");
        // suspend for 0.5 or 2 sec, depending on _errcount
        suspend(1000, 5000, 5);
		return;
	}

    Debug_printf("Setting up BeckerPort: listening on %s:%d\n", _host, _port);

    // Create listening socket
    _listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (_listen_fd < 0)
    {
        Debug_printf("BeckerPort: failed to create socket: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        suspend(BECKER_SUSPEND_MS);
		return;
	}

    // Set socket option
    int enable = 1;
#if defined(_WIN32)
    if (setsockopt(_listen_fd, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (char *) &enable, sizeof(enable)) != 0)
#else
    if (setsockopt(_listen_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) != 0)
#endif
    {
        Debug_printf("BeckerPort: setsockopt failed: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        closesocket(_listen_fd);
        _listen_fd = -1;
        suspend(BECKER_SUSPEND_MS);
        return;
    }

    // Local address to listen on
    if (_host[0] == '\0' || !strcmp(_host, "*"))
    {
        _ip = IPADDR_ANY;
    }
    else
    {
        _ip = get_ip4_addr_by_name(_host);
        if (_ip == IPADDR_NONE)
        {
            Debug_println("BeckerPort: failed to resolve host name");
            closesocket(_listen_fd);
            _listen_fd = -1;
            suspend(BECKER_SUSPEND_MS);
            return;
        }
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = _ip;
    addr.sin_port = htons(_port);

    // Bind to listening address
    if (bind(_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        Debug_printf("BeckerPort: bind failed: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        closesocket(_listen_fd);
        _listen_fd = -1;
        suspend(BECKER_SUSPEND_MS);
        return;
    }

    // Listen for incoming connection
    if (listen(_listen_fd, 1) != 0)
    {
        Debug_printf("BeckerPort: listen failed: %d  %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        closesocket(_listen_fd);
        _listen_fd = -1;
        suspend(BECKER_SUSPEND_MS);
        return;
    }

    // Set socket non-blocking
    if (!compat_socket_set_nonblocking(_listen_fd))
    {
        Debug_printf("BeckerPort: failed to set non-blocking mode: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        closesocket(_listen_fd);
        _listen_fd = -1;
        suspend(BECKER_SUSPEND_MS);
        return;
    }

    // Finally setup
    _errcount = 0; // used by suspend()
    setState(BeckerWaitConn::getInstance());
    Debug_printf("### BeckerPort accepting connections ###\n");
}

void BeckerPort::make_connection()
{

    // Wait for WiFi
    if (!fnWiFi.connected())
    {
        Debug_println("BeckerPort: No WiFi!");
        // suspend for 0.5 or 2 sec, depending on _errcount
        suspend(1000, 5000, 5);
		return;
	}

    Debug_printf("Setting up BeckerPort: connecting to %s:%d\n", _host, _port);

    // Create connection socket
    _fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (_fd < 0)
    {
        Debug_printf("BeckerPort: failed to create socket: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        suspend(BECKER_SUSPEND_MS);
		return;
	}

#ifdef USE_SO_NOSIGPIPE
    // Set NOSIGPIPE socket option (old macOS)
    int enable = 1;
    if (setsockopt(_fd, SOL_SOCKET, SO_NOSIGPIPE, (char *)&enable, sizeof(enable)) < 0)
    {
        Debug_printf("BeckerPort: setsockopt failed: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        suspend(BECKER_SUSPEND_MS);
        return;
    }
#endif

    // Set socket non-blocking
    if (!compat_socket_set_nonblocking(_fd))
    {
        Debug_printf("BeckerPort: failed to set non-blocking mode: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        suspend(BECKER_SUSPEND_MS);
        return;
    }

    // Remote address
    if (_host[0] == '\0')
    {
        _ip = IPADDR_LOOPBACK;
    }
    else
    {
        _ip = get_ip4_addr_by_name(_host);
        if (_ip == IPADDR_NONE)
        {
            Debug_println("BeckerPort: failed to resolve host name");
            suspend(BECKER_SUSPEND_MS);
            return;
        }
    }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = _ip;
    addr.sin_port = htons(_port);

    // Connect to remote address
    int res = connect(_fd, (struct sockaddr *)&addr, sizeof(addr));
    int err = compat_getsockerr();

#if defined(_WIN32)
    if (res < 0 && err != WSAEWOULDBLOCK)
#else
    if (res < 0 && err != EINPROGRESS)
#endif
    {
        Debug_printf("BeckerPort: connect failed: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        suspend(BECKER_SUSPEND_MS);
        return;
    }

    if (!wait_sock_writable(BECKER_CONNECT_TMOUT))
    {
        Debug_printf("BeckerPort: socket not ready\n");
        suspend(BECKER_SUSPEND_MS);
        return;
    }

    // Finally setup
    _errcount = 0;
    setState(BeckerConnected::getInstance());
    Debug_print("### BeckerPort connected ###\n");
}


bool BeckerPort::accept_pending_connection(int ms)
{
    // if listening socket has new connection accept it
    return(wait_sock_readable(ms, true) && accept_connection());
}


bool BeckerPort::accept_connection()
{
    struct sockaddr_in addr;
    int as = sizeof(struct sockaddr_in);

    // Accept connection
    _fd = accept(_listen_fd, (struct sockaddr *)&addr, (socklen_t *)&as);
    if (_fd < 0)
    {
        Debug_printf("BeckerPort: accept failed: %d - %s\n",
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        return false;
    }
    Debug_printf("BeckerPort: connection from: %s\r\n", inet_ntoa(addr.sin_addr));

    // Set socket options
    int val = 1;
    if (setsockopt(_fd, SOL_SOCKET, SO_KEEPALIVE, (char *)&val, sizeof(val)) < 0)
    {
        Debug_printf("BeckerPort warning: failed to set KEEPALIVE on socket\n");
    }
    if (setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, (char *)&val, sizeof(val)) < 0)
    {
        Debug_printf("BeckerPort warning: failed to set NODELAY on socket\n");
    }

    // Set socket non-blocking
    if (!compat_socket_set_nonblocking(_fd))
    {
        Debug_printf("BeckerPort: failed to set non-blocking connection: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        shutdown(_fd, 0);
        closesocket(_fd);
        _fd = -1;
        return false;
    }

    // We are connected !
    Debug_print("### BeckerPort connected ###\n");
    setState(BeckerConnected::getInstance());
    return true;
}

void BeckerPort::suspend(int short_ms, int long_ms, int threshold)
{
    if (_fd >= 0)
    {
        closesocket(_fd);
        _fd  = -1;
    }
    _errcount++;
    _suspend_time = fnSystem.millis();
    _suspend_period = short_ms;
    if (threshold > 0 && _errcount > threshold && long_ms > 0)
        _suspend_period = long_ms;
    Debug_printf("Suspending BeckerPort for %d ms\n", _suspend_period);
    setState(BeckerSuspended::getInstance());
}

void BeckerPort::suspend_on_disconnect()
{
    if (_listening && _listen_fd >=0)
    {
        if (_fd >= 0)
        {
            closesocket(_fd);
            _fd = -1;
        }
        // go directly into waiting for connection state
        setState(BeckerWaitConn::getInstance());
    }
    else
    {
        // wait before reconnecting
        suspend(BECKER_SUSPEND_MS);
    }
}

bool BeckerPort::resume()
{
    // Debug_print("Resuming BeckerPort\n");
    if (_listening)
    {
        if (_listen_fd >= 0)
        {
            // go directly into waiting for connection state
            setState(BeckerWaitConn::getInstance());
            return true;
        }

    }
    // listen or connect
    start_connection();
    return (_state != &BeckerSuspended::getInstance());
}

bool BeckerPort::suspend_period_expired()
{
    return (fnSystem.millis() - _suspend_time > _suspend_period);
}

bool BeckerPort::connected()
{
    uint8_t dummy;
    bool con = false;
    int res = recv(_fd, (char *)&dummy, 1, MSG_PEEK);
    if (res > 0)
    {
        con = true;
    }
    else if (res == 0)
    {
        Debug_print("### BeckerPort disconnected ###\n");
    }
    else
    {
        int err = compat_getsockerr();
        switch (err)
        {
#if defined(_WIN32)
        case WSAEWOULDBLOCK:
#else
        case EWOULDBLOCK:
        case ENOENT: // Caused by VFS
#endif
            con = true;
            break;
        default:
            Debug_printf("BeckerPort: connection error: %d - %s\n", 
                compat_getsockerr(), compat_sockstrerror(err));
            break;
        }
    }
    return con;
}

bool BeckerPort::poll_connection(int ms)
{
    if (wait_sock_readable(ms) && !connected())
    {
        // connection was closed or it has an error
        suspend_on_disconnect();
    }
    return false;
}

timeval BeckerPort::timeval_from_ms(const uint32_t millis)
{
  timeval tv;
  tv.tv_sec = millis / 1000;
  tv.tv_usec = (millis - (tv.tv_sec * 1000)) * 1000;
  return tv;
}

size_t BeckerPort::do_read(uint8_t *buffer, size_t size)
{
    int result;
    int to_recv;
    int rxbytes = 0;

    while (rxbytes < size)
    {
        to_recv = size - rxbytes;
        result = read_sock(buffer+rxbytes, to_recv);
        if (result > 0)
            rxbytes += result;
        else // result <= 0 for disconnected or write error
            break;
    }
    return rxbytes;
}

ssize_t BeckerPort::do_write(const uint8_t *buffer, size_t size)
{
    int result;
    int to_send;
    int txbytes = 0;

    while (txbytes < size)
    {
        to_send = size - txbytes;
        result = write_sock(buffer+txbytes, to_send);
        if (result > 0)
            txbytes += result;
        else if (result < 0) // write error
            break;
    }
    return txbytes;
}

ssize_t BeckerPort::read_sock(const uint8_t *buffer, size_t size, uint32_t timeout_ms)
{
    if (!wait_sock_readable(timeout_ms))
    {
        Debug_printf("BeckerPort: read_sock() TIMEOUT\n");
        return -1;
    }

    ssize_t result = recv(_fd, (char *)buffer, size, 0);
    if (result < 0)
    {
        Debug_printf("BeckerPort: read_sock() error: %d - %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        suspend_on_disconnect();
    }
    else if (result == 0)
    {
        Debug_printf("BeckerPort disconnected\n");
        suspend_on_disconnect();
    }
    return result;
}

ssize_t BeckerPort::write_sock(const uint8_t *buffer, size_t size, uint32_t timeout_ms)
{
    if (!wait_sock_writable(timeout_ms))
    {
        int err = compat_getsockerr();
#if defined(_WIN32)
        if (err == WSAETIMEDOUT)
#else
        if (err == ETIMEDOUT)
#endif
        {
            Debug_printf("BeckerPort: write_sock() TIMEOUT\n");
        }
        else
        {
            suspend_on_disconnect();
        }
        return -1;
    }

    ssize_t result = send(_fd, (char *)buffer, size, 0);
    if (result < 0)
    {
        Debug_printf("BeckerPort write_sock() error %d: %s\n", 
            compat_getsockerr(), compat_sockstrerror(compat_getsockerr()));
        suspend_on_disconnect();
    }
    return result;
}

bool BeckerPort::wait_sock_readable(uint32_t timeout_ms, bool listener)
{
    timeval timeout_tv;
    fd_set readfds;
    int result;
    int fd = listener ? _listen_fd : _fd;

    for(;;)
    {
        // Setup a select call to block for socket data or a timeout
        timeout_tv = timeval_from_ms(timeout_ms);
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        result = select(fd + 1, &readfds, nullptr, nullptr, &timeout_tv);

        // select error
        if (result < 0)
        {
            int err = compat_getsockerr();
#if defined(_WIN32)
            if (err == WSAEINTR)
#else
            if (err == EINTR) 
#endif
            {
                // TODO adjust timeout_tv
                continue;
            }
            Debug_printf("BeckerPort: wait_sock_readable() select error %d: %s\n", err, compat_sockstrerror(err));
            return false;
        }

        // select timeout
        if (result == 0)
            return false;

        // this shouldn't happen, if result > 0 our fd has to be in the list!
        if (!FD_ISSET(fd, &readfds))
        {
            Debug_println("BeckerPort: wait_sock_readable() unexpected select result");
            return false;
        }
        break;
    }
    return true;
}

bool BeckerPort::wait_sock_writable(uint32_t timeout_ms)
{
    timeval timeout_tv;
    fd_set writefds;
    fd_set errfds;
    int result;

    for(;;)
    {
        timeout_tv = timeval_from_ms(timeout_ms);
        // select for write
        FD_ZERO(&writefds);
        FD_SET(_fd, &writefds);
        // select for error too
        FD_ZERO(&errfds);
        FD_SET(_fd, &errfds);

        result = select(_fd + 1, nullptr, &writefds, &errfds, &timeout_tv);

        // select error
        if (result < 0) 
        {
            int err = compat_getsockerr();
#if defined(_WIN32)
            if (err == WSAEINTR)
#else
            if (err == EINTR) 
#endif
            {
                // TODO adjust timeout_tv
                continue;
            }
            Debug_printf("BeckerPort wait_sock_writable() select error %d: %s\n", err, compat_sockstrerror(err));
            return false;
        }

        // select timeout
        if (result == 0)
        {
#if defined(_WIN32)
            int err = WSAETIMEDOUT;
#else
            int err = ETIMEDOUT;
#endif
            // set errno
            compat_setsockerr(err);
            return false;
        }
        // Check for error on socket
        {
            int sockerr;
            socklen_t len = (socklen_t)sizeof(int);
            // Store any socket error value in sockerr
            int res = getsockopt(_fd, SOL_SOCKET, SO_ERROR, (char *)&sockerr, &len);
            if (res < 0)
            {
                // Failed to retrieve SO_ERROR
                int err = compat_getsockerr();
                Debug_printf("getsockopt on fd %d, errno: %d - %s\n", _fd, err ,compat_sockstrerror(err));
                return false;
            }
            // Retrieved SO_ERROR and found that we have an error condition
            if (sockerr != 0)
            {
                Debug_printf("socket error on fd %d, errno: %d - %s\n", _fd, sockerr, compat_sockstrerror(sockerr));
                // set errno
                compat_setsockerr(sockerr);
                return false;
            }
        }
        //Debug_print("socket is ready for write\n");
        break;
    }
    return true;
}

//
// Becker state handlers
//

// Stopped state

bool BeckerStopped::poll(BeckerPort *port, int ms)
{
#ifndef ESP_PLATFORM
    fnSystem.delay(ms); // be nice to CPU
#endif
    return false;
}

size_t BeckerStopped::read(BeckerPort *port, uint8_t *buffer, size_t size)
{
    // read timeout
    fnSystem.delay(BECKER_IOWAIT_MS);
    return 0;
}

ssize_t BeckerStopped::write(BeckerPort *port, const uint8_t *buffer, size_t size)
{
    // write timeout
    fnSystem.delay(BECKER_IOWAIT_MS);
    return 0;
}

// Suspended state

bool BeckerSuspended::poll(BeckerPort *port, int ms)
{
    if (!port->suspend_period_expired())
    {
        // still suspended
#ifndef ESP_PLATFORM
        fnSystem.delay(ms); // be nice to CPU
#endif
        return false;
    }
    // resume
    return port->resume();
}

size_t BeckerSuspended::read(BeckerPort *port, uint8_t *buffer, size_t size)
{
    if (!port->suspend_period_expired())
    {
        // still suspended, read timeout
        fnSystem.delay(BECKER_IOWAIT_MS);
        return 0;
    }
    // resume
    if (port->resume())
    {
        // connection was resumed, we can proceed with read
        return port->getState()->read(port, buffer, size);
    }
    return 0;
}

ssize_t BeckerSuspended::write(BeckerPort *port, const uint8_t *buffer, size_t size)
{
    if (!port->suspend_period_expired())
    {
        // still suspended, read timeout
        fnSystem.delay(BECKER_IOWAIT_MS);
        return 0;
    }
    // resume
    if (port->resume())
    {
        // connection was resumed, we can proceed with write
        return port->getState()->write(port, buffer, size);
    }
    return 0;
}

// Waiting for connection

bool BeckerWaitConn::poll(BeckerPort *port, int ms)
{
    return port->accept_pending_connection(ms); // true if new connection was accepted
}

size_t BeckerWaitConn::read(BeckerPort *port, uint8_t *buffer, size_t size)
{
    if (port->accept_pending_connection(BECKER_IOWAIT_MS))
    {
        // connection was accepted, we can proceed with read
        return port->getState()->read(port, buffer, size);
    }
    return 0;
}

ssize_t BeckerWaitConn::write(BeckerPort *port, const uint8_t *buffer, size_t size)
{
    if (port->accept_pending_connection(BECKER_IOWAIT_MS))
    {
        // connection was accepted, we can proceed with write
        return port->getState()->write(port, buffer, size);
    }
    return 0;
}

// Connected

bool BeckerConnected::poll(BeckerPort *port, int ms)
{
    return port->poll_connection(ms);
}

size_t BeckerConnected::read(BeckerPort *port, uint8_t *buffer, size_t size)
{
    return port->do_read(buffer, size);
}

ssize_t BeckerConnected::write(BeckerPort *port, const uint8_t *buffer, size_t size)
{
    return port->do_write(buffer, size);
}

#endif // BUILD_COCO