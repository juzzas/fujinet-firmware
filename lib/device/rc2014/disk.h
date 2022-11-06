#ifndef DISK_H
#define DISK_H

#include "bus.h"
#include "media.h"

#define STATUS_OK        0
#define STATUS_BAD_BLOCK 1
#define STATUS_NO_BLOCK  2
#define STATUS_NO_MEDIA  3
#define STATUS_NO_DRIVE  4

class rc2014Disk : public virtualDevice
{
private:
    MediaType *_media = nullptr;
    TaskHandle_t diskTask;

    unsigned long blockNum=INVALID_SECTOR_VALUE;

    void set_status(uint8_t s);
    void rc2014_control_clr();
    void rc2014_control_receive();
    void rc2014_control_send();
    void rc2014_control_send_block_num();
    void rc2014_control_send_block_data();
    virtual void rc2014_response_status();
    void rc2014_response_send();

    void rc2014_process(uint32_t commanddata, uint8_t checksum) override;

public:
    rc2014Disk();
    mediatype_t mount(FILE *f, const char *filename, uint32_t disksize, mediatype_t disk_type = MEDIATYPE_UNKNOWN);
    void unmount();
    bool write_blank(FILE *f, uint32_t numBlocks);
    virtual void reset();

    mediatype_t mediatype() { return _media == nullptr ? MEDIATYPE_UNKNOWN : _media->_mediatype; };

    bool device_active = false;

    ~rc2014Disk();
};

#endif /* s100_DISK_H */