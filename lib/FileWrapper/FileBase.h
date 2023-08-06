//
// Created by justin on 06/08/23.
//

#ifndef FILEBASE_H
#define FILEBASE_H

#include <array>
#include <cstdio>
#include <string>

#include "fujiHost.h"

class FileBase{
public:
    FileBase(fujiHost* host, std::string const& filename);
    ~FileBase();

    void open(fujiHost* host, std::string const& filename);
    void close();
    size_t read(uint8_t* buffer, size_t buffer_size);
    size_t write(uint8_t* buffer, size_t buffer_size);

    std::string& fullpath();
    bool isOpen();
    size_t filesize();
    fpos_t position();

private:
    FILE* m_file;
    std::string m_full_path;
    std::array<uint8_t, 1024> m_buffer;
    fujiHost* m_host;
};


#endif //FILEBASE_H
