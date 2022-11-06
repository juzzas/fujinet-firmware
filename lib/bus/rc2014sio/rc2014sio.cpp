#ifdef BUILD_RC2014

/**
 * rc2014 Functions
 */
#include "rc2014sio.h"

#include "../../include/debug.h"

#include "fnSystem.h"
#include "led.h"

uint8_t rc2014_checksum(uint8_t *buf, unsigned short len)
{
    uint8_t checksum = 0x00;

    for (unsigned short i = 0; i < len; i++)
        checksum ^= buf[i];

    return checksum;
}


void virtualDevice::rc2014_send(uint8_t b)
{
    fnUartSIO.write(b);
}

void virtualDevice::rc2014_flush()
{
    fnUartSIO.flush();
}


void virtualDevice::rc2014_send_buffer(uint8_t *buf, unsigned short len)
{
}

uint8_t virtualDevice::rc2014_recv()
{
    while (fnUartSIO.available() <= 0)
        fnSystem.yield();

    return fnUartSIO.read();
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
    return 0;
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
    rc2014_send('A');
    rc2014_flush();
}

void virtualDevice::rc2014_response_nack()
{
    rc2014_send('N');
    rc2014_flush();
}

void virtualDevice::rc2014_send_complete()
{
    rc2014_send('C');
    rc2014_flush();
}

void virtualDevice::rc2014_send_error()
{
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

void virtualDevice::rc2014_process(uint32_t commanddata, uint8_t checksum)
{
    cmdFrame.commanddata = commanddata;
    cmdFrame.checksum = checksum;


    fnUartDebug.printf("rc2014_process() not implemented yet for this device. Cmd received: %02x\n", cmdFrame.comnd);
}

void virtualDevice::rc2014_control_status()
{
}

void virtualDevice::rc2014_response_status()
{
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
    cmdFrame_t tempFrame;
    tempFrame.commanddata = 0;
    tempFrame.checksum = 0;

    if (fnUartSIO.readBytes((uint8_t *)&tempFrame, sizeof(tempFrame)) != sizeof(tempFrame))
    {
        Debug_println("Timeout waiting for data after CMD pin asserted");
        return;
    }
    // Turn on the RS232 indicator LED
    fnLedManager.set(eLed::LED_BUS, true);

    Debug_printf("\nCF: %02x %02x %02x %02x %02x\n",
                 tempFrame.device, tempFrame.comnd, tempFrame.aux1, tempFrame.aux2, tempFrame.cksum);
    // Wait for CMD line to raise again
    while (fnSystem.digital_read(PIN_CMD) == DIGI_LOW)
        vTaskDelay(1);

    uint8_t ck = rc2014_checksum((uint8_t *)&tempFrame.commanddata, sizeof(tempFrame.commanddata)); // Calculate Checksum
    if (ck == tempFrame.checksum)
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
            {
                // find device, ack and pass control
                // or go back to WAIT
                for (auto devicep : _daisyChain)
                {
                    if (tempFrame.device == devicep.second->_devnum)
                    {
                        _activeDev = devicep.second;
                        // handle command
                        _activeDev->rc2014_process(tempFrame.commanddata, tempFrame.checksum);
                    }
                }
            }
        }
    } // valid checksum
    else
    {
        Debug_printf("CHECKSUM_ERROR: Calc checksum: %02x\n",ck);
        // Switch to/from hispeed RS232 if we get enough failed frame checksums
    }
    fnLedManager.set(eLed::LED_BUS, false);
}

void systemBus::_rc2014_process_queue()
{
}

void systemBus::service()
{
}

void systemBus::setup()
{
    Debug_println("RC2014 SETUP");
// Set up UART
    fnUartSIO.begin(RC2014SIO_BAUDRATE);

    // CMD PIN
    fnSystem.set_pin_mode(PIN_CMD, gpio_mode_t::GPIO_MODE_INPUT); // There's no PULLUP/PULLDOWN on pins 34-39
    fnSystem.set_pin_mode(PIN_RS232_RTS, gpio_mode_t::GPIO_MODE_INPUT);
    fnSystem.set_pin_mode(PIN_RS232_CTS, gpio_mode_t::GPIO_MODE_OUTPUT);
    fnSystem.digital_write(PIN_RS232_CTS,DIGI_LOW);

    // Create a message queue
    //qRs232Messages = xQueueCreate(4, sizeof(rs232_message_t));

    Debug_println("RC2014 Setup Flush");
    fnUartSIO.flush_input();
}

void systemBus::shutdown()
{
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

    switch (device_id)
    {
    case RC2014_DEVICEID_FUJINET:
        _fujiDev = (rc2014Fuji *)pDevice;
        break;
    }
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

systemBus rc2014Bus;
#endif /* NEW_TARGET */