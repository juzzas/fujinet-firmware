//
// Created by justin on 06/08/23.
//

#include "FileBinary.h"

FileBinary::FileBinary(fujiHost* host, std::string const& filename, File::Mode mode) :
        FileBase(host, filename, mode)
{
}

FileBinary::~FileBinary()
{
    FileBase::~FileBase();
}

File::RC FileBinary::open(fujiHost* host, std::string const& filename, File::Mode mode)
{
    return FileBase::open(host, filename, mode);
}

File::RC FileBinary::close()
{
    return FileBase::close();
}

File::RC FileBinary::read(uint8_t* buffer, uint32_t buffer_size)
{
    return FileBase::read(buffer, buffer_size);
}

File::RC FileBinary::write(uint8_t* buffer, uint32_t buffer_size)
{
    return FileBase::write(buffer, buffer_size);
}

File::RC FileBinary::position(uint32_t *result)
{
    return FileBase::position(result);
}

File::RC FileBinary::filesize(uint32_t *result)
{
    return FileBase::filesize(result);
}
