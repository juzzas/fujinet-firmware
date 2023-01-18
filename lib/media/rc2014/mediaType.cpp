#ifdef BUILD_RC2014

#include "mediaType.h"

#include <cstdint>
#include <cstring>


MediaType::~MediaType()
{
    unmount();
}

bool MediaType::format(uint16_t *respopnsesize)
{
    return true;
}

bool MediaType::read(uint16_t sectornum, uint16_t *readcount)
{
    return true;
}

bool MediaType::write(uint16_t sectornum, bool verify)
{
    return true;
}

void MediaType::unmount()
{
    if (_media_fileh != nullptr)
    {
        fclose(_media_fileh);
        _media_fileh = nullptr;
    }
}

mediatype_t MediaType::discover_mediatype(const char *filename)
{
    int l = strlen(filename);
    if (l > 4 && filename[l - 4] == '.')
    {
        // Check the last 3 characters of the string
        const char *ext = filename + l - 3;
        if (strcasecmp(ext, "IMG") == 0)
        {
            return MEDIATYPE_IMG;
        }
    }
    return MEDIATYPE_UNKNOWN;
}

uint16_t MediaType::sector_size(uint16_t sector)
{
    (void)sector; // variable sector lengths are evil!

    return DISK_BYTES_PER_SECTOR_SINGLE;
}

#endif // NEW_TARGET