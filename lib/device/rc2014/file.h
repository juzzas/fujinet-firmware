#ifndef FILE_H
#define FILE_H

#include <array>

#include "bus.h"

class rc2014File : public virtualDevice
{
private:
    TaskHandle_t diskTask;
    std::string m_file_path;
    FILE* m_file;
    std::array<uint8_t, 1024> m_buffer;
    int m_host_id;

    void open();
    void close();
    void seek();
    void read();
    void write();
    void status();

    void rc2014_process(uint32_t commanddata, uint8_t checksum) override;

public:
    rc2014File();
    ~rc2014File();

    bool device_active = false;
};

#endif /* FILE_H */
