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

class FileBinary : public FileBase {
public:

    FileBinary(fujiHost* host, std::string const& filename, File::Mode mode);
    ~FileBinary();

    virtual File::RC open(fujiHost* host, std::string const& filename, File::Mode mode);
    virtual File::RC close() override;

    virtual File::RC read(uint8_t* buffer, uint32_t buffer_size) override;
    virtual File::RC write(uint8_t* buffer, uint32_t buffer_size) override;

    virtual File::RC filesize(uint32_t *result) override;
    virtual File::RC position(uint32_t *result) override;

private:
    std::string m_full_path;
    std::array<uint8_t, 1024> m_buffer;
};


#endif //FILEBASE_H
