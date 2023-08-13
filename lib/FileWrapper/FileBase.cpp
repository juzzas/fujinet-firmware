//
// Created by justin on 06/08/23.
//

#include "FileBase.h"

FileBase::FileBase(fujiHost* host, std::string const& filename, File::Mode mode) :
        m_file(nullptr),
        m_current_pos(0),
        m_mode(mode)
{
    open(host, filename);
}

FileBase::~FileBase()
{
    close();
}

File::RC FileBase::open(fujiHost* host, std::string const& filename, File::Mode mode)
{
    if (is_open()) {
        close();
    }

    char fullpath[MAX_PATHLEN];
    m_file = host->file_open(filename.c_str(), fullpath, MAX_PATHLEN, "rb");
    m_host = host;
    m_full_path = std::string(fullpath);

    if (!m_file)
        return File::RC::ERR_NOT_OPEN;

    m_current_pos = 0;

    return File::RC::OK;
}

File::RC FileBase::close()
{
    if (m_file) {
        fclose(m_file);
        m_file = nullptr;
        m_host = nullptr;
    }

    return File::RC::OK;
}

File::RC FileBase::seek(uint32_t pos)
{
    int rc = fseek(m_file, (off_t)pos, SEEK_SET);
    if (rc == 0) {
        m_current_pos = pos;
        return File::RC::OK;
    }

    return File::RC::ERR_FAIL_SEEK;
}

size_t FileBase::buffer_read(uint32_t offset, uint8_t* buffer, uint32_t buffer_size)
{
    File::RC rc = seek(offset);
    if (rc == File::RC::OK) {
        return fread(buffer, 1, buffer_size, m_file);
    }

    return 0;
}

File::RC FileBase::read(uint8_t* buffer, uint32_t buffer_size)
{
    size_t read = buffer_read(m_current_pos, buffer, buffer_size);
    if (read == buffer_size) {
        m_current_pos += buffer_size;
        return File::RC::OK;
    }

    return File::RC::ERR_SHORT_READ;
}

size_t FileBase::buffer_write(uint32_t offset, uint8_t* buffer, uint32_t buffer_size)
{
    File::RC rc = seek(offset);
    if (rc == File::RC::OK) {
        return fwrite(buffer, 1, buffer_size, m_file);
    }

    return rc;
}

File::RC FileBase::write(uint8_t* buffer, uint32_t buffer_size)
{
    size_t wrote = buffer_write(m_current_pos, buffer, buffer_size);
    if (wrote == buffer_size) {
        m_current_pos += buffer_size;
        return File::RC::OK;
    }

    return File::RC::ERR_SHORT_READ;
}

std::string& FileBase::fullpath()
{
    return m_full_path;
}

bool FileBase::is_open()
{
    return (m_file != nullptr);
}

uint32_t FileBase::filesize()
{
    return m_host->file_size(m_file);
}

uint32_t FileBase::position()
{
    return m_current_pos;
}

uint32_t FileBase::read_available()
{
    return filesize() - position();
}

bool FileBase::read_eof()
{
    return position() < filesize();
}
