#ifndef DISK_H
#define DISK_H

#include <fujiHost.h>
#include "bus.h"
#include "media.h"

class sioDisk : public virtualDevice
{
private:
    MediaType *_disk = nullptr;

    void sio_read();
    void sio_write(bool verify);
    void sio_format();
    void sio_status() override;
    void sio_process(uint32_t commanddata, uint8_t checksum) override;

    void derive_percom_block(uint16_t numSectors);
    void sio_read_percom_block();
    void sio_write_percom_block();
    void dump_percom_block();

public:
    sioDisk();
    fujiHost *host;
    mediatype_t mount(fnFile *f, const char *filename, uint32_t disksize, mediatype_t disk_type = MEDIATYPE_UNKNOWN);
    void unmount();
    bool write_blank(fnFile *f, uint16_t sectorSize, uint16_t numSectors);

    mediatype_t disktype() { return _disk == nullptr ? MEDIATYPE_UNKNOWN : _disk->_disktype; };

    ~sioDisk();
};

#endif
