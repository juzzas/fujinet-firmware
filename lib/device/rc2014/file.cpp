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
    m_file = nullptr;
}

// Destructor
rc2014File::~rc2014File()
{
    if (m_file != nullptr)
        fclose(m_file);
}

void rc2014File::open()
{
    Debug_print("FILE OPEN\n");
    uint8_t mode = cmdFrame.aux1;
    uint8_t host_id = cmdFrame.aux2;

    if (m_file != nullptr) {
        rc2014_send_error();
        return;
    }

    m_host_id = host_id;
    Debug_printf("FILE OPEN: host %d\n", host_id);

    fujiHost* host = theFuji.get_hosts(host_id);
    if (!host) {
        rc2014_send_error();
        return;
    }

    rc2014_send_ack();

    rc2014_recv_buffer(m_buffer.data(), 256);
    m_file_path = std::string(reinterpret_cast<char*>(m_buffer.data()));
    Debug_printf("FILE OPEN: %s\n", m_file_path.c_str());

    char fullpath[MAX_PATHLEN];
    m_file = host->file_open(m_file_path.c_str(), fullpath, MAX_PATHLEN, "rb");
    Debug_printf("FILE OPEN: %s\n", fullpath);

    if (m_file) {
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

    if (m_file != nullptr)
    {
        fclose(m_file);
        m_file = nullptr;
    }

    rc2014_send_complete();
}

// Read disk data and send to computer
void rc2014File::read()
{
    Debug_print("FILE READ\n");

    if (m_file == nullptr)
    {
        rc2014_send_error();
        return;
    }

    rc2014_send_ack();

    // Send result to RC2014
    uint16_t buffer_size = UINT16_FROM_HILOBYTES(cmdFrame.aux2, cmdFrame.aux1);
    size_t sending = fread(m_buffer.data(), 1, buffer_size, m_file);
    if (sending != buffer_size) {
        Debug_printf("FILE READ: requested %d, read %d\n", buffer_size, sending);
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

    if (m_file != nullptr) {
        uint16_t buffer_size = UINT16_FROM_HILOBYTES(cmdFrame.aux2, cmdFrame.aux1);

        rc2014_recv_buffer(m_buffer.data(), buffer_size);
        rc2014_send_ack();

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

    uint8_t status[10] = {};

    if (m_file) {
        status[0] = 0x01; // file status opened

        fujiHost* host = theFuji.get_hosts(m_host_id);
        uint32_t file_size = host->file_size(m_file);
        Debug_printf("FILE STATUS: size = %u\n", file_size);

        // file_size in Z80 little-endian order
        status[2] = file_size & 0xff; // file size
        status[3] = (file_size >> 8) & 0xff;
        status[4] = (file_size >> 16) & 0xff;
        status[5] = (file_size >> 24) & 0xff;

        fpos_t file_pos;
        int rc = fgetpos(m_file, &file_pos);
        if (rc == 0) {
            Debug_printf("FILE STATUS: position = %u\n", file_pos);
            status[6] = file_pos & 0xff; // file position
            status[7] = (file_pos >> 8) & 0xff;
            status[8] = (file_pos >> 16) & 0xff;
            status[9] = (file_pos >> 24) & 0xff;
        }
    }

    status[1] = 0x00; // error code

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
