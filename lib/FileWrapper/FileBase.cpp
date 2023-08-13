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

File::RC FileBase::buffer_read(uint32_t offset, uint8_t* buffer, uint32_t buffer_size)
{
    File::RC rc = seek(offset);

    if (rc == File::RC::OK) {
        size_t read = fread(buffer, 1, buffer_size, m_file);
        if (read < buffer_size) {
            return File::RC::ERR_SHORT_READ;
        }
    }

    return rc;
}

File::RC FileBase::read(uint8_t* buffer, uint32_t buffer_size)
{
    File::RC rc = buffer_read(m_current_pos, buffer, buffer_size);

    if (rc == File::RC::OK)
        m_current_pos += buffer_size;

    return rc;
}

File::RC FileBase::buffer_write(uint32_t offset, uint8_t* buffer, uint32_t buffer_size)
{
    File::RC rc = seek(offset);

    if (rc == File::RC::OK) {
        size_t wrote = fwrite(buffer, 1, buffer_size, m_file);

        if (wrote < buffer_size) {
            return File::RC::ERR_SHORT_WRITE;
        }

    }

    return rc;
}

File::RC FileBase::write(uint8_t* buffer, uint32_t buffer_size)
{
    File::RC rc = buffer_write(m_current_pos, buffer, buffer_size);

    if (rc == File::RC::OK)
        m_current_pos += buffer_size;

    return rc;
}

std::string& FileBase::fullpath()
{
    return m_full_path;
}

bool FileBase::is_open()
{
    return (m_file != nullptr);
}

File::RC FileBase::filesize(uint32_t *result)
{
    *result = m_host->file_size(m_file);
    return File::RC::OK;
}

File::RC FileBase::position(uint32_t *result)
{
    *result = m_current_pos;
    return File::RC::OK;
}

