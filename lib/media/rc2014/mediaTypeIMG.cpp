#ifdef BUILD_RC2014

#include "mediaTypeIMG.h"

#include <cstdint>
#include <cstring>
#include <utility>

#include "../../include/debug.h"

// Returns byte offset of given sector number (1-based)
uint32_t MediaTypeIMG::_sector_to_offset(uint16_t sectorNum)
{
    return (uint32_t )sectorNum * DISK_BYTES_PER_SECTOR_SINGLE;
}

// Returns TRUE if an error condition occurred
bool MediaTypeIMG::read(uint16_t sectornum, uint16_t *readcount)
{
    Debug_print("IMG READ\n");

    *readcount = 0;

    // Return an error if we're trying to read beyond the end of the disk
    if (sectornum > _media_num_sectors)
    {
        Debug_printf("::read sector %d > %d\n", sectornum, _media_num_sectors);
        return true;
    }

    uint16_t sectorSize = DISK_BYTES_PER_SECTOR_SINGLE;

    memset(_media_sectorbuff, 0, sizeof(_media_sectorbuff));

    bool err = false;
    // Perform a seek if we're not reading the sector after the last one we read
    if (sectornum != _media_last_sector + 1)
    {
        uint32_t offset = _sector_to_offset(sectornum);
        err = fseek(_media_fileh, offset, SEEK_SET) != 0;
    }

    if (err == false)
        err = fread(_media_sectorbuff, 1, sectorSize, _media_fileh) != sectorSize;

    if (err == false)
        _media_last_sector = sectornum;
    else
        _media_last_sector = INVALID_SECTOR_VALUE;

    *readcount = sectorSize;

    return err;
}

// Returns TRUE if an error condition occurred
bool MediaTypeIMG::write(uint16_t sectornum, bool verify)
{
    Debug_printf("IMG WRITE %u of %u\n", sectornum, _media_num_sectors);

    // Return an error if we're trying to write beyond the end of the disk
    if (sectornum > _media_num_sectors)
    {
        Debug_printf("::write sector %d > %d\n", sectornum, _media_num_sectors);
        _media_controller_status=2;
        return true;
    }

    uint32_t offset = _sector_to_offset(sectornum);

    _media_last_sector = INVALID_SECTOR_VALUE;

    // Perform a seek if we're writing to the sector after the last one
    int e;
    if (sectornum != _media_last_sector + 1)
    {
        e = fseek(_media_fileh, offset, SEEK_SET);
        if (e != 0)
        {
            Debug_printf("::write seek error %d\n", e);
            return true;
        }
    }
    // Write the data
    e = fwrite(_media_sectorbuff, 1, DISK_BYTES_PER_SECTOR_SINGLE, _media_fileh);
    if (e != DISK_BYTES_PER_SECTOR_SINGLE)
    {
        Debug_printf("::write error %d, %d\n", e, errno);
        return true;
    }

    int ret = fflush(_media_fileh);    // This doesn't seem to be connected to anything in ESP-IDF VF, so it may not do anything
    ret = fsync(fileno(_media_fileh)); // Since we might get reset at any moment, go ahead and sync the file (not clear if fflush does this)
    Debug_printf("IMG::write fsync:%d\n", ret);

    _media_last_sector = sectornum;
    _media_controller_status=0;

    return false;
}

void MediaTypeIMG::status(uint8_t statusbuff[4])
{
}

/*
    From Altirra manual:
    The format command formats a disk, writing 40 tracks and then verifying all sectors.
    All sectors are filleded with the data byte $00. On completion, the drive returns
    a sector-sized buffer containing a list of 16-bit bad sector numbers terminated by $FFFF.
*/
// Returns TRUE if an error condition occurred
bool MediaTypeIMG::format(uint16_t *responsesize)
{
    Debug_print("IMG FORMAT\n");

    // Populate an empty bad sector map
    memset(_media_sectorbuff, 0, sizeof(_media_sectorbuff));

    *responsesize = _media_sector_size;

    return false;
}

/* 
 Mount 8MB RC2014 CP/M "slice"
*/
mediatype_t MediaTypeIMG::mount(FILE *f, uint32_t disksize)
{
    Debug_print("IMG MOUNT\n");

    _media_fileh = f;
    _media_num_sectors = disksize / 512;
    _mediatype = MEDIATYPE_IMG;

    return _mediatype;
}

// Returns FALSE on error
bool MediaTypeIMG::create(FILE *f, uint16_t sectorSize, uint16_t numSectors)
{
    return true;
}

#endif /* BUILD_ADAM */
