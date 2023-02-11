#if defined(BUILD_RC2014) && defined(RC2014_BUS_SPI)

/**
 * rc2014 Functions
 */
#include "rc2014bus.h"

#include "../../include/debug.h"
#include "driver/spi_slave.h"


#include "fnConfig.h"
#include "fnSystem.h"

#include "led.h"
#include "modem.h" 

#define RC2014_SPI_HOST   SPI3_HOST


uint8_t rc2014_checksum(uint8_t *buf, unsigned short len)
{
    uint8_t checksum = 0x00;

    for (unsigned short i = 0; i < len; i++)
        checksum ^= buf[i];

    return checksum;
}


rc2014Buffer::rc2014Buffer(unsigned size) :
        size_(size)
{
    buffer_.reserve(size + 1);
}

bool rc2014Buffer::validate()
{
    if (buffer_.size() == 0)
        return true;

    uint8_t ck = rc2014_checksum(buffer_.data(), buffer_.size() - 1);

    return ck == buffer_.back();
}

uint8_t* rc2014Buffer::data()
{
    return buffer_.data();
}

size_t rc2014Buffer::max_size()
{
    return size_;
}

size_t rc2014Buffer::data_size()
{
    return buffer_.size();
}

uint8_t rc2014Command::device()
{
    return buffer_[0];
}

uint8_t rc2014Command::command()
{
    return buffer_[1];
}

uint16_t rc2014Command::aux()
{
    return (buffer_[3] << 8) + buffer_[2];
}

uint8_t rc2014Command::aux1()
{
    return buffer_[2];
}

uint8_t rc2014Command::aux2()
{
    return buffer_[3];
}

uint8_t rc2014Command::checksum()
{
    return buffer_[4];
}

cmdFrame_t* rc2014Command::frame()
{
    return (cmdFrame_t*)buffer_.data();
}

void virtualDevice::rc2014_send(uint8_t b)
{
    rc2014Bus.busTxByte(b);
}

void virtualDevice::rc2014_send_string(const std::string& str)
{
    for (auto& c: str) {
        rc2014_send(c);
    }
}

void virtualDevice::rc2014_send_int(const int i)
{
    rc2014_send_string(std::to_string(i));
}

void virtualDevice::rc2014_flush()
{
    rc2014Bus.busTxTransfer();
}


size_t virtualDevice::rc2014_send_buffer(const uint8_t *buf, unsigned short len)
{
    for (int i = 0; i < len; i++) {
        //Debug_printf("[0x%02x] ", buf[i]);
        rc2014_send(buf[i]);
    }
    //Debug_printf("\n");

    return len;
}


uint8_t virtualDevice::rc2014_recv()
{
    uint8_t val;

    spi_slave_transaction_t t;

    t.length=8;   // bits
    t.tx_buffer=&val;
    t.rx_buffer=&val;

    /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
    initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
    by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
    .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
    data.
    */
    esp_err_t rc = spi_slave_transmit(RC2014_SPI_HOST, &t, portMAX_DELAY);

    return val;
}

int virtualDevice::rc2014_recv_available()
{
    return 0;
}

bool virtualDevice::rc2014_recv_timeout(uint8_t *b, uint64_t dur)
{
    return false;
}

uint16_t virtualDevice::rc2014_recv_length()
{
    unsigned short s = 0;
    s = rc2014_recv() << 8;
    s |= rc2014_recv();

    return s;
}

void virtualDevice::rc2014_send_length(uint16_t l)
{
    rc2014_send(l >> 8);
    rc2014_send(l & 0xFF);
}

unsigned short virtualDevice::rc2014_recv_buffer(uint8_t *buf, unsigned short len)
{
    return rc2014Bus.busRxBuffer(buf, len);
}

uint32_t virtualDevice::rc2014_recv_blockno()
{
    unsigned char x[4] = {0x00, 0x00, 0x00, 0x00};

    rc2014_recv_buffer(x, 4);

    return x[3] << 24 | x[2] << 16 | x[1] << 8 | x[0];
}

void virtualDevice::reset()
{
    Debug_printf("No Reset implemented for device %u\n", _devnum);
}

void virtualDevice::rc2014_response_ack()
{
    Debug_println("ACK!");
    rc2014_send('A');
    rc2014_flush();
}

void virtualDevice::rc2014_response_nack()
{
    Debug_println("NAK!");
    rc2014_send('N');
    rc2014_flush();
}

void virtualDevice::rc2014_send_complete()
{
    Debug_println("COMPLETE!");
    rc2014_send('C');
    rc2014_flush();
}

void virtualDevice::rc2014_send_error()
{
    Debug_println("ERROR!");
    rc2014_send('E');
    rc2014_flush();
}

void virtualDevice::rc2014_control_ready()
{
    rc2014_response_ack();
}

void systemBus::wait_for_idle()
{
}

void virtualDevice::rc2014_process(rc2014Command& cmdFrame)
{
    fnUartDebug.printf("rc2014_process() not implemented yet for this device. Cmd received: %02x\n", cmdFrame.command());
}

void virtualDevice::rc2014_control_status()
{
}

void virtualDevice::rc2014_response_status()
{
}

void virtualDevice::rc2014_handle_stream()
{
    //fnUartSIO.flush_input();
}

void virtualDevice::rc2014_idle()
{
    // Not implemented in base class
}

//void virtualDevice::rc2014_status()
//{
//    fnUartDebug.printf("rc2014_status() not implemented yet for this device.\n");
//}

void systemBus::_rc2014_process_cmd()
{
    Debug_printf("rc2014_process_cmd()\n");

        // Read CMD frame
    rc2014Command tempFrame;


    size_t bytes_read = busRxBuffer(tempFrame.data(), tempFrame.max_size());
    Debug_printf("(bytes_read = %d)\n", (int)bytes_read);

    if (bytes_read != tempFrame.max_size())
    {
        Debug_printf("Timeout waiting for data after CMD pin asserted (bytes_read = %d)\n", (int)bytes_read);
        //gpio_set_level(PIN_CMD_RDY, DIGI_HIGH);
        return;
    }

    // Turn on the RC2014 BUS indicator LED
    fnLedManager.set(eLed::LED_BUS, true);

    Debug_printf("\nCF: %02x %02x %02x %02x %02x\n",
                 tempFrame.device(), tempFrame.command(), tempFrame.aux1(), tempFrame.aux2(), tempFrame.checksum());

    if (tempFrame.validate())
    {
#if 0
        if (tempFrame.device == RC2014_DEVICEID_DISK && _fujiDev != nullptr && _fujiDev->boot_config)
        {
            _activeDev = _fujiDev->bootdisk();
            if (_activeDev->status_wait_count > 0 && tempFrame.comnd == 'R' && _fujiDev->status_wait_enabled)
            {
                Debug_printf("Disabling CONFIG boot.\n");
                _fujiDev->boot_config = false;
                return;
            }
            else
            {
                Debug_println("FujiNet CONFIG boot");
                // handle command
                _activeDev->rc2014_process(tempFrame.commanddata, tempFrame.checksum);
            }
        }
        else
#endif
        {
            // find device, ack and pass control
            // or go back to WAIT
            auto devp = _daisyChain.find(tempFrame.device());
            if (devp != _daisyChain.end()) {
                (*devp).second->rc2014_process(tempFrame);
            }
            else
            {
                Debug_printf("CF for unknown device (%d)\n", tempFrame.device());
            }
        }
    } // valid checksum
    else
    {
        Debug_printf("CHECKSUM_ERROR: Calc checksum: %02x\n", tempFrame.checksum());
        // Switch to/from hispeed RS232 if we get enough failed frame checksums
    }

    // Wait for CMD line to raise again
    while (fnSystem.digital_read(PIN_CMD) == DIGI_LOW)
        vTaskDelay(1);


    fnLedManager.set(eLed::LED_BUS, false);
}

void systemBus::_rc2014_process_queue()
{
}

void systemBus::service()
{
#if 0
    if (_cpmDev != nullptr && _cpmDev->cpmActive)
    {
        _cpmDev->rc2014_handle_cpm();
        return; // break!
    }    
#endif
    // Go process a command frame if the RS232 CMD line is asserted
    if (fnSystem.digital_read(PIN_CMD) == DIGI_LOW)
    {
        Debug_println("RC2014 CMD low");
        _rc2014_process_cmd();
    }
#if 0
    // Go check if the modem needs to read data if it's active
    else if (_streamDev != nullptr && _streamDev->device_active)
    {
        _streamDev->rc2014_handle_stream();
    }
    else
    // Neither CMD nor active streaming device, so throw out any stray input data
    {
        //Debug_println("RS232 Srvc Flush");
        //fnUartSIO.flush_input();
    }

    // Handle interrupts from network protocols
    for (int i = 0; i < 8; i++)
    {
        if (_netDev[i] != nullptr)
            _netDev[i]->rs232_poll_interrupt();
    }
#endif
}

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
// Note: called after master asserts CS.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(PIN_CMD_RDY, DIGI_LOW);
    Debug_println("RC2014 SPI setup cb");
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    gpio_set_level(PIN_CMD_RDY, DIGI_HIGH);
    Debug_println("RC2014 SPI done cb");
}


void systemBus::setup()
{
    Debug_println("RC2014 SETUP");
// Set up UART
    //fnUartSIO.begin(RC2014SIO_BAUDRATE);

    // CMD PIN
    fnSystem.set_pin_mode(PIN_CMD, gpio_mode_t::GPIO_MODE_INPUT); // There's no PULLUP/PULLDOWN on pins 34-39

    fnSystem.set_pin_mode(PIN_CMD_RDY, gpio_mode_t::GPIO_MODE_OUTPUT);
    fnSystem.digital_write(PIN_CMD_RDY, DIGI_HIGH);


    // Set up SPI bus
    spi_bus_config_t bus_cfg = 
    {
        .mosi_io_num = PIN_BUS_DEVICE_MOSI,
        .miso_io_num = PIN_BUS_DEVICE_MISO,
        .sclk_io_num = PIN_BUS_DEVICE_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
        .flags=0,
        .intr_flags=0,
    };

    spi_slave_interface_config_t slave_cfg =
    {
        .spics_io_num=PIN_BUS_DEVICE_CS,
        .flags=0,
        .queue_size=1,
        .mode=0,
        .post_setup_cb=my_post_setup_cb,
        .post_trans_cb=my_post_trans_cb
    };

    esp_err_t rc = spi_slave_initialize(RC2014_SPI_HOST, &bus_cfg, &slave_cfg, SPI_DMA_DISABLED);

    // Create a message queue
    //qRs232Messages = xQueueCreate(4, sizeof(rs232_message_t));

    Debug_println("RC2014 Setup Flush");
}

void systemBus::shutdown()
{
    shuttingDown = true;

    for (auto devicep : _daisyChain)
    {
        Debug_printf("Shutting down device %02x\n", devicep.second->id());
        devicep.second->shutdown();
    }
    Debug_printf("All devices shut down.\n");
}

void systemBus::addDevice(virtualDevice *pDevice, uint8_t device_id)
{
    Debug_printf("Adding device: %02X\n", device_id);
    pDevice->_devnum = device_id;
    _daisyChain[device_id] = pDevice;
}

bool systemBus::deviceExists(uint8_t device_id)
{
    return _daisyChain.find(device_id) != _daisyChain.end();
}

void systemBus::remDevice(virtualDevice *pDevice)
{

}

void systemBus::remDevice(uint8_t device_id)
{
    if (deviceExists(device_id))
    {
        _daisyChain.erase(device_id);
    }
}

int systemBus::numDevices()
{
    return _daisyChain.size();
}

void systemBus::changeDeviceId(virtualDevice *p, uint8_t device_id)
{
    for (auto devicep : _daisyChain)
    {
        if (devicep.second == p)
            devicep.second->_devnum = device_id;
    }
}

virtualDevice *systemBus::deviceById(uint8_t device_id)
{
    for (auto devicep : _daisyChain)
    {
        if (devicep.second->_devnum == device_id)
            return devicep.second;
    }
    return nullptr;
}

void systemBus::reset()
{
    for (auto devicep : _daisyChain)
        devicep.second->reset();
}

void systemBus::enableDevice(uint8_t device_id)
{
    if (_daisyChain.find(device_id) != _daisyChain.end())
        _daisyChain[device_id]->device_active = true;
}

void systemBus::disableDevice(uint8_t device_id)
{
    if (_daisyChain.find(device_id) != _daisyChain.end())
        _daisyChain[device_id]->device_active = false;
}

bool systemBus::enabledDeviceStatus(uint8_t device_id)
{
    if (_daisyChain.find(device_id) != _daisyChain.end())
        return _daisyChain[device_id]->device_active;

    return false;
}

void systemBus::streamDevice(uint8_t device_id)
{
    Debug_printf("streamDevice: %x\n", device_id);
    auto device = _daisyChain.find(device_id);
    if (device != _daisyChain.end()) {
        if (device->second->device_active)
            _streamDev = device->second;
    }
}

void systemBus::streamDeactivate()
{
    _streamDev = nullptr;
}

size_t systemBus::busTxBuffer(const uint8_t *buf, unsigned short len)
{
    for (unsigned short i = 0; i < len; i++)
        _tx_buffer.push_back(buf[i]);

    return len;
}

size_t systemBus::busTxByte(const uint8_t byte)
{
    _tx_buffer.push_back(byte);

    return 1;
}

size_t systemBus::busTxTransfer()
{
    spi_slave_transaction_t t = {};

    Debug_printf("systemBus::busTxBuffer = %d bytes\n", _tx_buffer.size());

    t.length = _tx_buffer.size() * 8;   // bits
    t.tx_buffer = _tx_buffer.data();
    t.rx_buffer = _tx_buffer.data();

    /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
    initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
    by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
    .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
    data.
    */
    //gpio_set_level(PIN_CMD_RDY, DIGI_LOW);
    esp_err_t rc = spi_slave_transmit(RC2014_SPI_HOST, &t, 200 /*portMAX_DELAY*/);
    //gpio_set_level(PIN_CMD_RDY, DIGI_HIGH);

    Debug_printf("systemBus::busTxBuffer trans length = %d\n", t.trans_len);
    if (rc != ESP_OK)
        return 0;

    _tx_buffer.clear();

    return t.trans_len / 8;
}

size_t systemBus::busRxBuffer(uint8_t *buf, unsigned short len)
{
    spi_slave_transaction_t t = {};

    Debug_println("systemBus::busRxBuffer");

    t.length = len * 8;   // bits
    t.tx_buffer = buf;
    t.rx_buffer = buf;

    //gpio_set_level(PIN_CMD_RDY, DIGI_LOW);
    esp_err_t rc = spi_slave_transmit(RC2014_SPI_HOST, &t, 200 /*portMAX_DELAY*/);
    //gpio_set_level(PIN_CMD_RDY, DIGI_HIGH);

    Debug_printf("systemBus::busRxBuffer trans length = %d\n", t.trans_len);
    if (rc != ESP_OK)
        return 0;

    return t.trans_len / 8;

}

systemBus rc2014Bus;
#endif /* NEW_TARGET */