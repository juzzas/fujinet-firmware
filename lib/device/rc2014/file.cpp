#ifdef BUILD_RC2014

#include "file.h"
#include "fuji.h"

#include <memory.h>
#include <string.h>

#include "../../include/debug.h"

#include "utils.h"

#define RC2014_FILECMD_OPEN 0x4f
#define RC2014_FILECMD_CLOSE 0x43
#define RC2014_FILECMD_READ 0x52
#define RC2014_FILECMD_WRITE 0x57
#define RC2014_FILECMD_STATUS 0x53
#define RC2014_FILECMD_SEEK 0x4b

rc2014File::rc2014File()
{
}

// Destructor
rc2014File::~rc2014File()
{
    m_file->close();
}

void rc2014File::open()
{
    Debug_print("FILE OPEN\n");
    uint8_t mode = cmdFrame.aux1;
    m_host_id = cmdFrame.aux2;

    Debug_printf("FILE OPEN: host %d\n", m_host_id);

    fujiHost* host = theFuji.get_hosts(m_host_id);
    if (!host) {
        rc2014_send_error();
        return;
    }

    rc2014_send_ack();

    rc2014_recv_buffer(m_buffer.data(), 256);
    m_file_path = std::string(reinterpret_cast<char*>(m_buffer.data()));
    Debug_printf("FILE OPEN: %s\n", m_file_path.c_str());

    m_file = FileWrapper::Open(host, m_file_path);
    //m_file = host->file_open(m_file_path.c_str(), fullpath, MAX_PATHLEN, "rb");
    Debug_printf("FILE OPEN: %s\n", m_file->fullpath());

    if (m_file->is_open()) {
        rc2014_send_ack();
    } else {
        Debug_printf("FILE OPEN: unable to open %s\n", m_file_path.c_str());
        rc2014_send_error();
    }

    rc2014_send_complete();
}

void rc2014File::close()
{
    Debug_print("FILE CLOSE\n");

    rc2014_send_ack();

    m_file->close();

    rc2014_send_complete();
}

// Read disk data and send to computer
void rc2014File::read()
{
    Debug_print("FILE READ\n");

    if (!m_file->is_open())
    {
        rc2014_send_error();
        return;
    }

    rc2014_send_ack();

    // Send result to RC2014
    uint16_t buffer_size = UINT16_FROM_HILOBYTES(cmdFrame.aux2, cmdFrame.aux1);
    if (File::RC rc = m_file->read(m_buffer.data(), buffer_size); rc != File::RC::OK) {
        Debug_printf("FILE READ: error %d\n", rc);
        rc2014_send_error();
        return;
    }

    Debug_printf("FILE READ: %d bytes\n", buffer_size);
    rc2014_send_buffer(m_buffer.data(), buffer_size);
    rc2014_flush();

    rc2014_send_complete();
}

// Write disk data from computer
void rc2014File::write()
{
    Debug_print("FILE WRITE\n");
    rc2014_send_ack();

    if (m_file->is_open()) {
        uint16_t buffer_size = UINT16_FROM_HILOBYTES(cmdFrame.aux2, cmdFrame.aux1);

        rc2014_recv_buffer(m_buffer.data(), buffer_size);
        rc2014_send_ack();

        size_t sending = m_file->write(m_buffer.data(), buffer_size);
        if (sending != buffer_size) {
            Debug_printf("FILE WRITE: requested %d, read %d\n", buffer_size, sending);
        }
        Debug_printf("FILE WRITE: %d bytes\n", buffer_size);

        rc2014_send_complete();
        return;
    }

    rc2014_send_error();
}

// Status
void rc2014File::status()
{
    Debug_print("FILE STATUS\n");
    rc2014_send_ack();

    uint8_t status[6] = {};

    if (m_file->is_open()) {
        status[0] = 0x01; // file status opened

        uint32_t available = m_file->read_available();
        Debug_printf("FILE STATUS: remaining = %u\n", available);

        // file_size in Z80 little-endian order
        status[2] = available & 0xff; // file size
        status[3] = (available >> 8) & 0xff;
        status[4] = (available >> 16) & 0xff;
        status[5] = (available >> 24) & 0xff;
    }

    status[1] = m_file->read_eof(); // error code

    rc2014_send_buffer(status, sizeof(status));
    rc2014_flush();

    rc2014_send_complete();
}

void rc2014File::rc2014_process(uint32_t commanddata, uint8_t checksum)
{
    cmdFrame.commanddata = commanddata;
    cmdFrame.checksum = checksum;

    switch (cmdFrame.comnd) {
    case RC2014_FILECMD_OPEN:
        open();
        return;
    case RC2014_FILECMD_CLOSE:
        close();
        return;
    case RC2014_FILECMD_READ:
        read();
        return;
    case RC2014_FILECMD_STATUS:
        status();
        return;
    case RC2014_FILECMD_WRITE:
        write();
        return;
    default:
        Debug_printf("rc2014_process() command not implemented. Cmd received: %02x\n", cmdFrame.comnd);
        rc2014_send_nak();
    }
}

#endif /* BUILD_RC2014 */
