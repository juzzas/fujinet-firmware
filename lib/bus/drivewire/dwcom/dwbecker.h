#ifndef DWBECKER_H
#define DWBECKER_H

#include <sys/time.h>
#include <vector>

#include "dwport.h"
#include "fnDNS.h"

#define BECKER_DEFAULT_PORT     65504
#define BECKER_IOWAIT_MS        500
#define BECKER_CONNECT_TMOUT    2000
#define BECKER_SUSPEND_MS       5000

class BeckerPort;

class BeckerState
{
public:
    virtual bool poll(BeckerPort *port, int ms) = 0;
    virtual size_t read(BeckerPort *port, uint8_t *buffer, size_t size) = 0;
    virtual ssize_t write(BeckerPort *port, const uint8_t *buffer, size_t size) = 0;
    virtual ~BeckerState() {}
};

class BeckerStopped : public BeckerState
{
public:
    virtual bool poll(BeckerPort *port, int ms) override;
    virtual size_t read(BeckerPort *port, uint8_t *buffer, size_t size) override;
    virtual ssize_t write(BeckerPort *port, const uint8_t *buffer, size_t size) override;
    static BeckerStopped& getInstance() { static BeckerStopped instance; return instance; }

private:
	BeckerStopped() {}
	BeckerStopped(const BeckerStopped& other);
	BeckerStopped& operator=(const BeckerStopped& other);
};

class BeckerWaitConn : public BeckerState
{
public:
    virtual bool poll(BeckerPort *port, int ms) override;
    virtual size_t read(BeckerPort *port, uint8_t *buffer, size_t size) override;
    virtual ssize_t write(BeckerPort *port, const uint8_t *buffer, size_t size) override;
    static BeckerWaitConn& getInstance() { static BeckerWaitConn instance; return instance; }

private:
	BeckerWaitConn() {}
	BeckerWaitConn(const BeckerWaitConn& other);
	BeckerWaitConn& operator=(const BeckerWaitConn& other);
};

class BeckerConnected : public BeckerState
{
public:
    virtual bool poll(BeckerPort *port, int ms) override;
    virtual size_t read(BeckerPort *port, uint8_t *buffer, size_t size) override;
    virtual ssize_t write(BeckerPort *port, const uint8_t *buffer, size_t size) override;
    static BeckerConnected& getInstance() { static BeckerConnected instance; return instance; }
private:
	BeckerConnected() {}
	BeckerConnected(const BeckerConnected& other);
	BeckerConnected& operator=(const BeckerConnected& other);
};

class BeckerSuspended : public BeckerState
{
public:
    virtual bool poll(BeckerPort *port, int ms) override;
    virtual size_t read(BeckerPort *port, uint8_t *buffer, size_t size) override;
    virtual ssize_t write(BeckerPort *port, const uint8_t *buffer, size_t size) override;
    static BeckerSuspended& getInstance() { static BeckerSuspended instance; return instance; }
private:
	BeckerSuspended() {}
	BeckerSuspended(const BeckerSuspended& other);
	BeckerSuspended& operator=(const BeckerSuspended& other);
};


class BeckerPort : public DwPort
{
private:
    char _host[64]; // TODO change to std::string
    in_addr_t _ip;
    uint16_t _port;

    uint32_t _baud;     // not used by Becker

    // is waiting for connection (listening) or connecting to?
    bool _listening;

    // file descriptors
    int _fd;
    int _listen_fd;

    // state machine handlers for poll(), read() and write()
    BeckerState *_state;

    // error counter
    int _errcount;
#ifdef ESP_PLATFORM
    unsigned long _suspend_time;
#else
    uint64_t _suspend_time;
#endif
    int _suspend_period;

protected:
	void start_connection();
	void listen_for_connection();
	void make_connection();
	bool accept_connection();

	void suspend(int short_ms, int long_ms=0, int threshold=0);
	void suspend_on_disconnect();
	bool resume();
	bool suspend_period_expired();

	bool accept_pending_connection(int ms);

	bool connected();
	bool poll_connection(int ms);

	static timeval timeval_from_ms(const uint32_t millis);

	size_t do_read(uint8_t *buffer, size_t size);
	ssize_t do_write(const uint8_t *buffer, size_t size);

    ssize_t read_sock(const uint8_t *buffer, size_t size, uint32_t timeout_ms=BECKER_IOWAIT_MS);
    ssize_t write_sock(const uint8_t *buffer, size_t size, uint32_t timeout_ms=BECKER_IOWAIT_MS);

	bool wait_sock_readable(uint32_t timeout_ms, bool listener=false);
    bool wait_sock_writable(uint32_t timeout_ms);

public:
    BeckerPort();
    virtual ~BeckerPort();
    virtual void begin(int baud) override;
	virtual void end() override;

    // mimic UARTManager, baudrate is not used by BeckerPort
    virtual void set_baudrate(uint32_t baud) override { _baud = baud; }
    virtual uint32_t get_baudrate() override { return _baud; }

    virtual int available() override;
    virtual void flush() override;
    virtual void flush_input() override;

    // keep BeckerPort alive
	virtual bool poll(int ms) override { return _state->poll(this, ms); }
    // read bytes into buffer
    virtual size_t read(uint8_t *buffer, size_t size) override { return _state->read(this, buffer, size); }
    // write buffer
    virtual ssize_t write(const uint8_t *buffer, size_t size) override { return _state->write(this, buffer, size); }

    // specific to BeckerPort
    void set_host(const char *host, int port);
    const char* get_host(int &port);

	inline BeckerState* getState() const { return _state; }
	void setState(BeckerState& state) { _state = &state; }

    // friends, state handlers
    friend class BeckerStopped;
    friend class BeckerWaitConn;
    friend class BeckerConnected;
    friend class BeckerSuspended;
};



#endif // DWBECKER_H
