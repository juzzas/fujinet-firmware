//
// Created by justin on 06/08/23.
//

#ifndef FILEBASE_H
#define FILEBASE_H

#include <array>
#include <string>

#include "fujiHost.h"

namespace File {
    enum RC {
        OK,
        ERR_NOT_IMPLEMENTED,
        ERR_NOT_OPEN,
        ERR_INVALID_PARAMS,
        ERR_SHORT_READ,
        ERR_SHORT_WRITE,
        ERR_FAIL_SEEK
    };

    enum class Mode : std::uint8_t {
        OREAD   = 0b00000001,
        OWRITE  = 0b00000010,
        OAPPEND = 0b00000100,
        OCREATE = 0b00001000,
        // ...
    };
}

class FileBase {
public:
    FileBase(fujiHost* host, std::string const& filename, File::Mode mode);
    ~FileBase();

    virtual File::RC open(fujiHost* host, std::string const& filename, File::Mode mode = File::Mode::OREAD);
    virtual File::RC close();

    virtual File::RC read(uint8_t* buffer, uint32_t buffer_size);
    virtual File::RC write(uint8_t* buffer, uint32_t buffer_size);
    virtual File::RC seek(uint32_t pos);

    virtual File::RC filesize(uint32_t *result);
    virtual File::RC position(uint32_t *result);

    std::string& fullpath();
    bool is_open();

protected:
    File::RC buffer_read(uint32_t offset, uint8_t* buffer, uint32_t buffer_size);
    File::RC buffer_write(uint32_t offset, uint8_t* buffer, uint32_t buffer_size);


private:
    FILE* m_file;
    std::string m_full_path;
    fujiHost* m_host;
    uint32_t m_current_pos;
    File::Mode m_mode;
};


#endif //FILEBASE_H
