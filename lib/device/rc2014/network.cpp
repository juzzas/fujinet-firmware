#ifdef BUILD_RC2014

/**
 * N: Firmware
 */

#include "network.h"

#include <cstring>
#include <algorithm>

#include "../../include/debug.h"

#include "utils.h"

#include "status_error_codes.h"
#include "TCP.h"
#include "UDP.h"
#include "Test.h"
#include "Telnet.h"
#include "TNFS.h"
#include "FTP.h"
#include "HTTP.h"
#include "SSH.h"
#include "SMB.h"

//using namespace std;

/**
 * Constructor
 */
rc2014Network::rc2014Network()
{
    status_response[1] = 0x00;
    status_response[2] = 0x04; // 1024 bytes
    status_response[3] = 0x00; // Character device

    receiveBuffer = new string();
    transmitBuffer = new string();
    specialBuffer = new string();

    receiveBuffer->clear();
    transmitBuffer->clear();
    specialBuffer->clear();
}

/**
 * Destructor
 */
rc2014Network::~rc2014Network()
{
    receiveBuffer->clear();
    transmitBuffer->clear();
    specialBuffer->clear();

    if (receiveBuffer != nullptr)
        delete receiveBuffer;
    if (transmitBuffer != nullptr)
        delete transmitBuffer;
    if (specialBuffer != nullptr)
        delete specialBuffer;
}

/** rc2014 COMMANDS ***************************************************************/

/**
 * rc2014 Open command
 * Called in response to 'O' command. Instantiate a protocol, pass URL to it, call its open
 * method. Also set up RX interrupt.
 */
void rc2014Network::open(unsigned short s)
{
    uint8_t _aux1 = rc2014_recv();
    uint8_t _aux2 = rc2014_recv();
    string d;

    s--; s--;
    
    memset(response,0,sizeof(response));
    rc2014_recv_buffer(response, s);
    rc2014_recv(); // checksum

    
    rc2014_response_ack();

    channelMode = PROTOCOL;

    // persist aux1/aux2 values
    cmdFrame.aux1 = _aux1;
    cmdFrame.aux2 = _aux2;

    open_aux1 = cmdFrame.aux1;
    open_aux2 = cmdFrame.aux2;

    // Shut down protocol if we are sending another open before we close.
    if (protocol != nullptr)
    {
        protocol->close();
        delete protocol;
        protocol = nullptr;
    }

    // Reset status buffer
    statusByte.byte = 0x00;

    Debug_printf("open()\n");

    // Parse and instantiate protocol
    d=string((char *)response,s);
    parse_and_instantiate_protocol(d);

    if (protocol == nullptr)
    {
        return;
    }

    // Attempt protocol open
    if (protocol->open(urlParser, &cmdFrame) == true)
    {
        statusByte.bits.client_error = true;
        Debug_printf("Protocol unable to make connection. Error: %d\n", err);
        delete protocol;
        protocol = nullptr;
        return;
    }
}

/**
 * rc2014 Close command
 * Tear down everything set up by open(), as well as RX interrupt.
 */
void rc2014Network::close()
{
    Debug_printf("rc2014Network::close()\n");

    rc2014_recv(); // CK

    
    rc2014_response_ack();

    statusByte.byte = 0x00;

    // If no protocol enabled, we just signal complete, and return.
    if (protocol == nullptr)
    {
        return;
    }

    // Ask the protocol to close
    protocol->close();

    // Delete the protocol object
    delete protocol;
    protocol = nullptr;
}

/**
 * Perform the channel read based on the channelMode
 * @param num_bytes - number of bytes to read from channel.
 * @return TRUE on error, FALSE on success. Passed directly to bus_to_computer().
 */
bool rc2014Network::read_channel(unsigned short num_bytes)
{
    bool _err = false;

    switch (channelMode)
    {
    case PROTOCOL:
        _err = protocol->read(num_bytes);
        break;
    case JSON:
        Debug_printf("JSON Not Handled.\n");
        _err = true;
        break;
    }
    return _err;
}

/**
 * rc2014 Write command
 * Write # of bytes specified by aux1/aux2 from tx_buffer out to rc2014. If protocol is unable to return requested
 * number of bytes, return ERROR.
 */
void rc2014Network::write(uint16_t num_bytes)
{
    memset(response, 0, sizeof(response));

    rc2014_recv_buffer(response, num_bytes);
    rc2014_recv(); // CK

    
    rc2014_response_ack();

    *transmitBuffer += string((char *)response, num_bytes);
    err = rc2014Network_write_channel(num_bytes);
}

/**
 * Perform the correct write based on value of channelMode
 * @param num_bytes Number of bytes to write.
 * @return TRUE on error, FALSE on success. Used to emit rc2014_error or rc2014_complete().
 */
bool rc2014Network::rc2014Network_write_channel(unsigned short num_bytes)
{
    bool err = false;

    switch (channelMode)
    {
    case PROTOCOL:
        err = protocol->write(num_bytes);
        break;
    case JSON:
        Debug_printf("JSON Not Handled.\n");
        err = true;
        break;
    }
    return err;
}

/**
 * rc2014 Status Command. First try to populate NetworkStatus object from protocol. If protocol not instantiated,
 * or Protocol does not want to fill status buffer (e.g. due to unknown aux1/aux2 values), then try to deal
 * with them locally. Then serialize resulting NetworkStatus object to rc2014.
 */
void rc2014Network::status()
{
    NetworkStatus s;
    rc2014_recv(); // CK
    
    rc2014_response_ack();

    switch (channelMode)
    {
    case PROTOCOL:
        err = protocol->status(&s);
        break;
    case JSON:
        // err = _json->status(&status);
        break;
    }

    response[0] = s.rxBytesWaiting & 0xFF;
    response[1] = s.rxBytesWaiting >> 8;
    response[2] = s.connected;
    response[3] = s.error;
    response_len = 4;
    receiveMode = STATUS;
}

/**
 * Get Prefix
 */
void rc2014Network::get_prefix()
{
    rc2014_recv(); // CK

    
    rc2014_response_ack();

    Debug_printf("rc2014Network::rc2014_getprefix(%s)\n", prefix.c_str());
    memcpy(response, prefix.data(), prefix.size());
    response_len = prefix.size();
}

/**
 * Set Prefix
 */
void rc2014Network::set_prefix(unsigned short s)
{
    uint8_t prefixSpec[256];
    string prefixSpec_str;

    memset(prefixSpec, 0, sizeof(prefixSpec));

    rc2014_recv_buffer(prefixSpec, s);
    rc2014_recv(); // CK

    
    rc2014_response_ack();

    prefixSpec_str = string((const char *)prefixSpec);
    prefixSpec_str = prefixSpec_str.substr(prefixSpec_str.find_first_of(":") + 1);
    Debug_printf("rc2014Network::rc2014_set_prefix(%s)\n", prefixSpec_str.c_str());

    if (prefixSpec_str == "..") // Devance path N:..
    {
        vector<int> pathLocations;
        for (int i = 0; i < prefix.size(); i++)
        {
            if (prefix[i] == '/')
            {
                pathLocations.push_back(i);
            }
        }

        if (prefix[prefix.size() - 1] == '/')
        {
            // Get rid of last path segment.
            pathLocations.pop_back();
        }

        // truncate to that location.
        prefix = prefix.substr(0, pathLocations.back() + 1);
    }
    else if (prefixSpec_str[0] == '/') // N:/DIR
    {
        prefix = prefixSpec_str;
    }
    else if (prefixSpec_str.empty())
    {
        prefix.clear();
    }
    else if (prefixSpec_str.find_first_of(":") != string::npos)
    {
        prefix = prefixSpec_str;
    }
    else // append to path.
    {
        prefix += prefixSpec_str;
    }

    Debug_printf("Prefix now: %s\n", prefix.c_str());
}

/**
 * Set login
 */
void rc2014Network::set_login(uint16_t s)
{
    uint8_t loginspec[256];

    memset(loginspec, 0, sizeof(loginspec));

    rc2014_recv_buffer(loginspec, s);
    rc2014_recv(); // ck

    
    rc2014_response_ack();

    login = string((char *)loginspec, s);
}

/**
 * Set password
 */
void rc2014Network::set_password(uint16_t s)
{
    uint8_t passwordspec[256];

    memset(passwordspec, 0, sizeof(passwordspec));

    rc2014_recv_buffer(passwordspec, s);
    rc2014_recv(); // ck

    
    rc2014_response_ack();

    password = string((char *)passwordspec, s);
}

void rc2014Network::del(uint16_t s)
{
    string d;

    memset(response,0,sizeof(response));
    rc2014_recv_buffer(response, s);
    rc2014_recv(); // CK

        
    rc2014_response_ack();

    d=string((char *)response,s);
    parse_and_instantiate_protocol(d);

    if (protocol == nullptr)
        return;

    cmdFrame.comnd = '!';

    if (protocol->perform_idempotent_80(urlParser, &cmdFrame))
    {
        statusByte.bits.client_error = true;
        return;
    }
}

void rc2014Network::rename(uint16_t s)
{
    string d;

    memset(response,0,sizeof(response));
    rc2014_recv_buffer(response, s);
    rc2014_recv(); // CK

    
    rc2014_response_ack();

    d=string((char *)response,s);
    parse_and_instantiate_protocol(d);

    cmdFrame.comnd = ' ';

    if (protocol->perform_idempotent_80(urlParser, &cmdFrame))
    {
        statusByte.bits.client_error = true;
        return;
    }
}

void rc2014Network::mkdir(uint16_t s)
{
    string d;

    memset(response,0,sizeof(response));
    rc2014_recv_buffer(response,s);
    rc2014_recv(); // CK

    
    rc2014_response_ack();

    d=string((char *)response,s);
    parse_and_instantiate_protocol(d);

    cmdFrame.comnd = '*';

    if (protocol->perform_idempotent_80(urlParser, &cmdFrame))
    {
        statusByte.bits.client_error = true;
        return;
    }
}

/**
 * rc2014 Special, called as a default for any other rc2014 command not processed by the other rc2014_ functions.
 * First, the protocol is asked whether it wants to process the command, and if so, the protocol will
 * process the special command. Otherwise, the command is handled locally. In either case, either rc2014_complete()
 * or rc2014_error() is called.
 */
void rc2014Network::rc2014Network_special()
{
    // do_inquiry(cmdFrame.comnd);

    // switch (inq_dstats)
    // {
    // case 0x00: // No payload
    //     rc2014_ack();
    //     rc2014_special_00();
    //     break;
    // case 0x40: // Payload to Atari
    //     rc2014_ack();
    //     rc2014_special_40();
    //     break;
    // case 0x80: // Payload to Peripheral
    //     rc2014_ack();
    //     rc2014_special_80();
    //     break;
    // default:
    //     rc2014_nak();
    //     break;
    // }
}

/**
 * @brief Do an inquiry to determine whether a protoocol supports a particular command.
 * The protocol will either return $00 - No Payload, $40 - Atari Read, $80 - Atari Write,
 * or $FF - Command not supported, which should then be used as a DSTATS value by the
 * Atari when making the N: rc2014 call.
 */
void rc2014Network::rc2014Network_special_inquiry()
{
    // // Acknowledge
    // rc2014_ack();

    // Debug_printf("rc2014Network::rc2014_special_inquiry(%02x)\n", cmdFrame.aux1);

    // do_inquiry(cmdFrame.aux1);

    // // Finally, return the completed inq_dstats value back to Atari
    // bus_to_computer(&inq_dstats, sizeof(inq_dstats), false); // never errors.
}

void rc2014Network::do_inquiry(unsigned char inq_cmd)
{
    // // Reset inq_dstats
    // inq_dstats = 0xff;

    // // Ask protocol for dstats, otherwise get it locally.
    // if (protocol != nullptr)
    //     inq_dstats = protocol->special_inquiry(inq_cmd);

    // // If we didn't get one from protocol, or unsupported, see if supported globally.
    // if (inq_dstats == 0xFF)
    // {
    //     switch (inq_cmd)
    //     {
    //     case 0x20:
    //     case 0x21:
    //     case 0x23:
    //     case 0x24:
    //     case 0x2A:
    //     case 0x2B:
    //     case 0x2C:
    //     case 0xFD:
    //     case 0xFE:
    //         inq_dstats = 0x80;
    //         break;
    //     case 0x30:
    //         inq_dstats = 0x40;
    //         break;
    //     case 'Z': // Set interrupt rate
    //         inq_dstats = 0x00;
    //         break;
    //     case 'T': // Set Translation
    //         inq_dstats = 0x00;
    //         break;
    //     case 0x80: // JSON Parse
    //         inq_dstats = 0x00;
    //         break;
    //     case 0x81: // JSON Query
    //         inq_dstats = 0x80;
    //         break;
    //     default:
    //         inq_dstats = 0xFF; // not supported
    //         break;
    //     }
    // }

    // Debug_printf("inq_dstats = %u\n", inq_dstats);
}

/**
 * @brief called to handle special protocol interactions when DSTATS=$00, meaning there is no payload.
 * Essentially, call the protocol action
 * and based on the return, signal rc2014_complete() or error().
 */
void rc2014Network::rc2014Network_special_00()
{
    // // Handle commands that exist outside of an open channel.
    // switch (cmdFrame.comnd)
    // {
    // case 'T':
    //     rc2014_set_translation();
    //     break;
    // case 'Z':
    //     rc2014_set_timer_rate();
    //     break;
    // default:
    //     if (protocol->special_00(&cmdFrame) == false)
    //         rc2014_complete();
    //     else
    //         rc2014_error();
    // }
}

/**
 * @brief called to handle protocol interactions when DSTATS=$40, meaning the payload is to go from
 * the peripheral back to the ATARI. Essentially, call the protocol action with the accrued special
 * buffer (containing the devicespec) and based on the return, use bus_to_computer() to transfer the
 * resulting data. Currently this is assumed to be a fixed 256 byte buffer.
 */
void rc2014Network::rc2014Network_special_40()
{
    // // Handle commands that exist outside of an open channel.
    // switch (cmdFrame.comnd)
    // {
    // case 0x30:
    //     rc2014_get_prefix();
    //     return;
    // }

    // bus_to_computer((uint8_t *)receiveBuffer->data(),
    //                 SPECIAL_BUFFER_SIZE,
    //                 protocol->special_40((uint8_t *)receiveBuffer->data(), SPECIAL_BUFFER_SIZE, &cmdFrame));
}

/**
 * @brief called to handle protocol interactions when DSTATS=$80, meaning the payload is to go from
 * the ATARI to the pheripheral. Essentially, call the protocol action with the accrued special
 * buffer (containing the devicespec) and based on the return, use bus_to_peripheral() to transfer the
 * resulting data. Currently this is assumed to be a fixed 256 byte buffer.
 */
void rc2014Network::rc2014Network_special_80()
{
    // uint8_t spData[SPECIAL_BUFFER_SIZE];

    // // Handle commands that exist outside of an open channel.
    // switch (cmdFrame.comnd)
    // {
    // case 0x20: // RENAME
    // case 0x21: // DELETE
    // case 0x23: // LOCK
    // case 0x24: // UNLOCK
    // case 0x2A: // MKDIR
    // case 0x2B: // RMDIR
    //     rc2014_do_idempotent_command_80();
    //     return;
    // case 0x2C: // CHDIR
    //     rc2014_set_prefix();
    //     return;
    // case 0xFD: // LOGIN
    //     rc2014_set_login();
    //     return;
    // case 0xFE: // PASSWORD
    //     rc2014_set_password();
    //     return;
    // }

    // memset(spData, 0, SPECIAL_BUFFER_SIZE);

    // // Get special (devicespec) from computer
    // bus_to_peripheral(spData, SPECIAL_BUFFER_SIZE);

    // Debug_printf("rc2014Network::rc2014_special_80() - %s\n", spData);

    // // Do protocol action and return
    // if (protocol->special_80(spData, SPECIAL_BUFFER_SIZE, &cmdFrame) == false)
    //     rc2014_complete();
    // else
    //     rc2014_error();
}

void rc2014Network::rc2014_response_status()
{
    NetworkStatus s;

    if (protocol != nullptr)
        protocol->status(&s);

    statusByte.bits.client_connected = s.connected == true;
    statusByte.bits.client_data_available = s.rxBytesWaiting > 0;
    statusByte.bits.client_error = s.error > 1;

    status_response[4] = statusByte.byte;
    virtualDevice::rc2014_response_status();
}

void rc2014Network::rc2014_control_ack()
{
}

void rc2014Network::rc2014_control_send()
{
    uint16_t s = rc2014_recv_length(); // receive length
    uint8_t c = rc2014_recv();        // receive command

    s--; // Because we've popped the command off the stack

    switch (c)
    {
    case ' ':
        rename(s);
        break;
    case '!':
        del(s);
        break;
    case '*':
        mkdir(s);
        break;
    case ',':
        set_prefix(s);
        break;
    case '0':
        get_prefix();
        break;
    case 'O':
        open(s);
        break;
    case 'C':
        close();
        break;
    case 'S':
        status();
        break;
    case 'W':
        write(s);
        break;
    case 0xFD: // login
        set_login(s);
        break;
    case 0xFE: // password
        set_password(s);
        break;
    default:
        Debug_printf("rc2014_control_send() - Unknown Command: %02x\n", c);
    }
}

void rc2014Network::rc2014_control_clr()
{
    rc2014_response_send();
}

void rc2014Network::rc2014_control_receive_channel()
{
    NetworkStatus ns;

    if ((protocol == nullptr) || (receiveBuffer == nullptr))
        return; // Punch out.

    // Get status
    protocol->status(&ns);

    if (ns.rxBytesWaiting > 0)
        rc2014_response_ack();
    else
    {
        rc2014_response_nack();
        return;
    }

    // Truncate bytes waiting to response size
    ns.rxBytesWaiting = (ns.rxBytesWaiting > 1024) ? 1024 : ns.rxBytesWaiting;
    response_len = ns.rxBytesWaiting;

    if (protocol->read(response_len)) // protocol adapter returned error
    {
        statusByte.bits.client_error = true;
        err = protocol->error;
        return;
    }
    else // everything ok
    {
        statusByte.bits.client_error = 0;
        statusByte.bits.client_data_available = response_len > 0;
        memcpy(response, receiveBuffer->data(), response_len);
        for (int i = 0; i < response_len; i++)
        {
            Debug_printf("%c", response[i]);
        }
        receiveBuffer->erase(0, response_len);
    }
}

void rc2014Network::rc2014_control_receive()
{
    

    if (response_len > 0) // There is response data, go ahead and ack.
    {
        rc2014_response_ack();
        return;
    }
    else if (protocol == nullptr)
    {
        rc2014_response_nack();
        return;
    }

    switch (receiveMode)
    {
    case CHANNEL:
        rc2014_control_receive_channel();
        break;
    case STATUS:
        break;
    }
}

void rc2014Network::rc2014_response_send()
{
    uint8_t c = rc2014_checksum(response, response_len);

    rc2014_send(0xB0 | _devnum);
    rc2014_send_length(response_len);
    rc2014_send_buffer(response, response_len);
    rc2014_send(c);

    memset(response, 0, response_len);
    response_len = 0;
}

/**
 * Process incoming rc2014 command
 * @param comanddata incoming 4 bytes containing command and aux bytes
 * @param checksum 8 bit checksum
 */
void rc2014Network::rc2014_process(uint32_t commanddata, uint8_t checksum)
{
    cmdFrame.commanddata = commanddata;
    cmdFrame.checksum = checksum;

    fnUartDebug.printf("rc2014_process() not implemented yet for this device. Cmd received: %02x\n", cmdFrame.comnd);
}

/** PRIVATE METHODS ************************************************************/

/**
 * Instantiate protocol object
 * @return bool TRUE if protocol successfully called open(), FALSE if protocol could not open
 */
bool rc2014Network::instantiate_protocol()
{
    if (urlParser == nullptr)
    {
        Debug_printf("rc2014Network::open_protocol() - urlParser is NULL. Aborting.\n");
        return false; // error.
    }

    // Convert to uppercase
    std::transform(urlParser->scheme.begin(), urlParser->scheme.end(), urlParser->scheme.begin(), ::toupper);

    if (urlParser->scheme == "TCP")
    {
        protocol = new NetworkProtocolTCP(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "UDP")
    {
        protocol = new NetworkProtocolUDP(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "TEST")
    {
        protocol = new NetworkProtocolTest(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "TELNET")
    {
        protocol = new NetworkProtocolTELNET(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "TNFS")
    {
        protocol = new NetworkProtocolTNFS(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "FTP")
    {
        protocol = new NetworkProtocolFTP(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "HTTP" || urlParser->scheme == "HTTPS")
    {
        protocol = new NetworkProtocolHTTP(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "SSH")
    {
        protocol = new NetworkProtocolSSH(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else if (urlParser->scheme == "SMB")
    {
        protocol = new NetworkProtocolSMB(receiveBuffer, transmitBuffer, specialBuffer);
    }
    else
    {
        Debug_printf("Invalid protocol: %s\n", urlParser->scheme.c_str());
        return false; // invalid protocol.
    }

    if (protocol == nullptr)
    {
        Debug_printf("rc2014Network::open_protocol() - Could not open protocol.\n");
        return false;
    }

    if (!login.empty())
    {
        protocol->login = &login;
        protocol->password = &password;
    }

    Debug_printf("rc2014Network::open_protocol() - Protocol %s opened.\n", urlParser->scheme.c_str());
    return true;
}

void rc2014Network::parse_and_instantiate_protocol(string d)
{
    deviceSpec = d;

    // Invalid URL returns error 165 in status.
    if (parseURL() == false)
    {
        Debug_printf("Invalid devicespec: %s\n", deviceSpec.c_str());
        statusByte.byte = 0x00;
        statusByte.bits.client_error = true;
        err = NETWORK_ERROR_INVALID_DEVICESPEC;
        return;
    }

    Debug_printf("Parse and instantiate protocol: %s\n", deviceSpec.c_str());

    // Instantiate protocol object.
    if (instantiate_protocol() == false)
    {
        Debug_printf("Could not open protocol.\n");
        statusByte.byte = 0x00;
        statusByte.bits.client_error = true;
        err = NETWORK_ERROR_GENERAL;
        return;
    }
}

/**
 * Is this a valid URL? (Used to generate ERROR 165)
 */
bool rc2014Network::isValidURL(EdUrlParser *url)
{
    if (url->scheme == "")
        return false;
    else if ((url->path == "") && (url->port == ""))
        return false;
    else
        return true;
}

/**
 * Preprocess deviceSpec given aux1 open mode. This is used to work around various assumptions that different
 * disk utility packages do when opening a device, such as adding wildcards for directory opens.
 *
 * The resulting URL is then sent into EdURLParser to get our URLParser object which is used in the rest
 * of rc2014Network.
 *
 * This function is a mess, because it has to be, maybe we can factor it out, later. -Thom
 */
bool rc2014Network::parseURL()
{
    string url;
    string unit = deviceSpec.substr(0, deviceSpec.find_first_of(":") + 1);

    if (urlParser != nullptr)
        delete urlParser;

    // Prepend prefix, if set.
    if (prefix.length() > 0)
        deviceSpec = unit + prefix + deviceSpec.substr(deviceSpec.find(":") + 1);
    else
        deviceSpec = unit + deviceSpec.substr(string(deviceSpec).find(":") + 1);

    Debug_printf("rc2014Network::parseURL(%s)\n", deviceSpec.c_str());

    // Strip non-ascii characters.
    util_strip_nonascii(deviceSpec);

    // Process comma from devicespec (DOS 2 COPY command)
    // processCommaFromDevicespec();

    if (cmdFrame.aux1 != 6) // Anything but a directory read...
    {
        std::replace(deviceSpec.begin(), deviceSpec.end(), '*', '\0'); // FIXME: Come back here and deal with WC's
    }

    // // Some FMSes add a dot at the end, remove it.
    // if (deviceSpec.substr(deviceSpec.length() - 1) == ".")
    //     deviceSpec.erase(deviceSpec.length() - 1, string::npos);

    // Remove any spurious spaces
    deviceSpec = util_remove_spaces(deviceSpec);

    // chop off front of device name for URL, and parse it.
    url = deviceSpec.substr(deviceSpec.find(":") + 1);
    urlParser = EdUrlParser::parseUrl(url);

    Debug_printf("rc2014Network::parseURL transformed to (%s, %s)\n", deviceSpec.c_str(), url.c_str());

    return isValidURL(urlParser);
}

void rc2014Network::rc2014Network_set_translation()
{
    // trans_aux2 = cmdFrame.aux2;
    // rc2014_complete();
}

void rc2014Network::rc2014Network_set_timer_rate()
{
    // timerRate = (cmdFrame.aux2 * 256) + cmdFrame.aux1;

    // // Stop extant timer
    // timer_stop();

    // // Restart timer if we're running a protocol.
    // if (protocol != nullptr)
    //     timer_start();

    // rc2014_complete();
}

void rc2014Network::rc2014Network_do_idempotent_command_80()
{
    // rc2014_ack();

    // parse_and_instantiate_protocol();

    // if (protocol == nullptr)
    // {
    //     Debug_printf("Protocol = NULL\n");
    //     rc2014_error();
    //     return;
    // }

    // if (protocol->perform_idempotent_80(urlParser, &cmdFrame) == true)
    // {
    //     Debug_printf("perform_idempotent_80 failed\n");
    //     rc2014_error();
    // }
    // else
    //     rc2014_complete();
}

#endif /* NEW_TARGET */