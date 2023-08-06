//
// Created by justin on 06/08/23.
//

#include "FileBase.h"

FileBase::FileBase(fujiHost* host, std::string const& filename) : m_file(nullptr)
{
    open(host, filename);
}

FileBase::~FileBase()
{
    if (m_file) {
        fclose(m_file);
    }
}

void FileBase::open(fujiHost* host, std::string const& filename)
{
    if (m_file) {
        close();
    }

    char fullpath[MAX_PATHLEN];
    m_file = host->file_open(filename.c_str(), fullpath, MAX_PATHLEN, "rb");
    m_host = host;
    m_full_path = std::string(fullpath);
}

void FileBase::close()
{
    if (m_file) {
        fclose(m_file);
        m_file = nullptr;
        m_host = nullptr;
    }
}

size_t FileBase::read(uint8_t* buffer, size_t buffer_size)
{
    return fread(buffer, 1, buffer_size, m_file);
}

size_t FileBase::write(uint8_t* buffer, size_t buffer_size)
{
    return fwrite(buffer, 1, buffer_size, m_file);
}

std::string& FileBase::fullpath()
{
    return m_full_path;
}

bool FileBase::isOpen()
{
    return (m_file != nullptr);
}

size_t FileBase::filesize()
{
    return m_host->file_size(m_file);
}

fpos_t FileBase::position()
{
    fpos_t file_pos;

    int rc = fgetpos(m_file, &file_pos);
    if (rc == 0)
    {
        return file_pos;
    }

    return 0;
}
