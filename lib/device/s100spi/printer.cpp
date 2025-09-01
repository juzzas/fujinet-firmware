#ifdef BUILD_S100

#include "printer.h"

#include <cstring>

#include "../../../include/debug.h"

#include "fnSystem.h"
#include "led.h"

#include "atari_1020.h"
#include "atari_1025.h"
#include "file_printer.h"
#include "html_printer.h"
#include "svg_plotter.h"
#include "epson_80.h"
#include "epson_tps.h"
#include "okimate_10.h"
#include "png_printer.h"
#include "coleco_printer.h"

std::string buf;
bool taskActive=false;

constexpr const char * const s100spiPrinter::printer_model_str[PRINTER_INVALID];

void printerTask(void *param)
{
    s100spiPrinter *ptr = (s100spiPrinter *)param;

    while(1)
    {
        if ((ptr != nullptr) && (ptr->bpos>0))
        {
            taskActive=true;
            ptr->getPrinterPtr()->process(ptr->bpos,0,0);
            ptr->bpos=0;
            taskActive=false;
        }

        vPortYield();
    }
}

// Constructor just sets a default printer type
s100spiPrinter::s100spiPrinter(FileSystem *filesystem, printer_type print_type)
{
    _storage = filesystem;
    set_printer_type(print_type);
}

s100spiPrinter::~s100spiPrinter()
{
    vTaskDelete(thPrinter);
    delete _pptr;
    _pptr = nullptr;
}

s100spiPrinter::printer_type s100spiPrinter::match_modelname(std::string model_name)
{
    int i;
    for (i = 0; i < PRINTER_INVALID; i++)
        if (model_name.compare(s100spiPrinter::printer_model_str[i]) == 0)
            break;

    return (printer_type)i;
}

void s100spiPrinter::s100spi_control_status()
{
    uint8_t c[6] = {0x82, 0x10, 0x00, 0x00, 0x00, 0x10};

    s100spi_send_buffer(c, sizeof(c));
}

void s100spiPrinter::idle()
{
    Debug_printf("s100spiPrinter::idle()\n");
    if (buf.empty())
        return;

    uint8_t c = buf.length() > 40 ? 40 : buf.length();

    fnLedManager.set(LED_BT,true);
    _last_ms=fnSystem.millis();

    memcpy(_pptr->provideBuffer(),buf.data(),c);
    _pptr->process(c,0,0);

    buf.erase(0,c);
    fnLedManager.set(LED_BT,false);
}

void s100spiPrinter::s100spi_control_send()
{
    unsigned short s = s100spi_recv_length();

    s100spi_recv_buffer(_buffer, s);
    uint8_t ck = s100spi_recv(); // ck

    
    s100spi_response_ack();

    _last_ms = fnSystem.millis();
    memcpy(_pptr->provideBuffer(),_buffer,s);
    bpos=s;
}

void s100spiPrinter::s100spi_control_ready()
{
    s100spi_response_ack();
}

void s100spiPrinter::s100spi_process(uint8_t b)
{
}

void s100spiPrinter::shutdown()
{
}

void s100spiPrinter::set_printer_type(printer_type printer_type)
{
    // Destroy any current printer emu object
    if (_pptr != nullptr)
    {
        delete _pptr;
    }

    _ptype = printer_type;
    switch (printer_type)
    {
    case PRINTER_FILE_RAW:
        _pptr = new filePrinter(RAW);
        break;
    case PRINTER_FILE_TRIM:
        _pptr = new filePrinter;
        break;
    case PRINTER_FILE_ASCII:
        _pptr = new filePrinter(ASCII);
        break;
    case PRINTER_ATARI_1020:
        _pptr = new atari1020;
        break;
    case PRINTER_ATARI_1025:
        _pptr = new atari1025;
        break;
    case PRINTER_EPSON:
        _pptr = new epson80;
        break;
    case PRINTER_EPSON_PRINTSHOP:
        _pptr = new epsonTPS;
        break;
    case PRINTER_OKIMATE10:
        _pptr = new okimate10;
        break;
    case PRINTER_PNG:
        _pptr = new pngPrinter;
        break;
    case PRINTER_HTML:
        _pptr = new htmlPrinter;
        break;
    case PRINTER_HTML_ATASCII:
        _pptr = new htmlPrinter(HTML_ATASCII);
        break;
    default:
        _pptr = new filePrinter;
        _ptype = PRINTER_FILE_TRIM;
        break;
    }

    _pptr->initPrinter(_storage);
}

#endif /* NEW_TARGET */