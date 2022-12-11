#ifndef _MEDIA_TYPE_
#define _MEDIA_TYPE_

#include <stdio.h>

#define INVALID_SECTOR_VALUE 0xFFFFFFFF

#define DISK_BYTES_PER_SECTOR_SINGLE 128
#define DISK_BYTES_PER_SECTOR_BLOCK 512

#define DISK_CTRL_STATUS_CLEAR 0x00

enum mediatype_t 
{
    MEDIATYPE_UNKNOWN = 0,
    MEDIATYPE_IMG,      // 8meg RomWBW disk splice  
    //MEDIATYPE_FD      // 1.44meg RomWBW floppy disk image
    //MEDIATYPE_DSK     // 720Kilobyte SAM Coupe Prodos floppy disk image
    MEDIATYPE_COUNT
};

class MediaType
{
protected:
    FILE *_media_fileh = nullptr;
    uint32_t _media_image_size = 0;
    uint32_t _media_num_sectors = 0;
    uint16_t _media_sector_size = DISK_BYTES_PER_SECTOR_SINGLE;

public:
// TODO: add CP/M DCB for image to pass to RC2014 CP/M BIOS/BDOS
//    struct
//    {
//        uint8_t num_tracks;
//        uint8_t step_rate;
//        uint8_t sectors_per_trackH;
//        uint8_t sectors_per_trackL;
//        uint8_t num_sides;
//        uint8_t density;
//        uint8_t sector_sizeH;
//        uint8_t sector_sizeL;
//        uint8_t drive_present;
//        uint8_t reserved1;
//        uint8_t reserved2;
//        uint8_t reserved3;
//    } _percomBlock;

    uint8_t _media_sectorbuff[DISK_BYTES_PER_SECTOR_SINGLE];
    uint32_t _media_last_sector = INVALID_SECTOR_VALUE-1;
    uint8_t _media_controller_status = DISK_CTRL_STATUS_CLEAR;

    mediatype_t _mediatype = MEDIATYPE_UNKNOWN;

    virtual mediatype_t mount(FILE *f, uint32_t disksize) = 0;
    virtual void unmount();

    // Returns TRUE if an error condition occurred
    virtual bool format(uint16_t *respopnsesize);

    // Returns TRUE if an error condition occurred
    virtual bool read(uint16_t sectornum, uint16_t *readcount) = 0;
    // Returns TRUE if an error condition occurred
    virtual bool write(uint16_t sectornum, bool verify);
    
    virtual void status(uint8_t statusbuff[4]) = 0;

    static mediatype_t discover_mediatype(const char *filename);
    uint16_t sector_size(uint16_t sector);

    virtual ~MediaType();
};

#endif // _MEDIA_TYPE_
