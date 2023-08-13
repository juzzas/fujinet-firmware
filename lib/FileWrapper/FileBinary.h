//
// Created by justin on 06/08/23.
//

#ifndef FILEBINARY_H
#define FILEBINARY_H

#include <array>
#include <cstdio>
#include <string>

#include "FileBase.h"

#include "fujiHost.h"

#define MAX_PRELOAD_BUFFER_LEN 1024

class FileBinary : public FileBase {
public:

    FileBinary(fujiHost* host, std::string const& filename, File::Mode mode);
    ~FileBinary();

    virtual File::RC open(fujiHost* host, std::string const& filename, File::Mode mode);

    virtual File::RC read(uint8_t* buffer, uint32_t buffer_size) override;
    virtual File::RC write(uint8_t* buffer, uint32_t buffer_size) override;
    uint32_t read_available() override;
    bool read_eof() override;

private:
    File::RC preload_buffer();
    uint32_t m_preload_buffer_pos;

    std::array<uint8_t, MAX_PRELOAD_BUFFER_LEN> m_buffer;
    uint32_t m_buffer_len;
};


#endif //FILEBASE_H
