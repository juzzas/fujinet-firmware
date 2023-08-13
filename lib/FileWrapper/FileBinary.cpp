//
// Created by justin on 06/08/23.
//

#include <cstring>
#include "FileBinary.h"

FileBinary::FileBinary(fujiHost* host, std::string const& filename, File::Mode mode) :
        FileBase(host, filename, mode),
        m_buffer_len(0)
{
}

FileBinary::~FileBinary()
{
    FileBase::~FileBase();
}

File::RC FileBinary::open(fujiHost* host, std::string const& filename, File::Mode mode)
{
    File::RC rc= FileBase::open(host, filename, mode);
    if (rc == File::RC::OK) {
        rc = preload_buffer();
    }

    if (rc == File::RC::OK) {
        rc = preload_buffer();
    }

    return rc;
}

File::RC FileBinary::read(uint8_t* buffer, uint32_t buffer_size)
{
    uint32_t to_read = buffer_size > m_buffer_len ? m_buffer_len : buffer_size;
    memcpy(buffer, m_buffer.data(), to_read);
    seek(m_preload_buffer_pos + m_buffer_len);

    return preload_buffer();
}

File::RC FileBinary::write(uint8_t* buffer, uint32_t buffer_size)
{
    return FileBase::write(buffer, buffer_size);
}

File::RC FileBinary::preload_buffer()
{
    m_preload_buffer_pos = position();

    size_t len = buffer_read(position(), m_buffer.data(), MAX_PRELOAD_BUFFER_LEN);
    if (len > 0) {
        m_buffer_len = len;
    } else {
        m_buffer_len = 0;
    }

    return File::RC::OK;
}

uint32_t FileBinary::read_available()
{
    return m_buffer_len;
}

bool FileBinary::read_eof()
{
    return m_buffer_len < MAX_PRELOAD_BUFFER_LEN;
}
