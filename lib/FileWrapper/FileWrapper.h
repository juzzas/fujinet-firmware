//
// Created by justin on 06/08/23.
//

#ifndef FILEWRAPPER_H
#define FILEWRAPPER_H

#include <memory>
#include "fujiHost.h"

#include "FileBinary.h"

struct FileWrapper{
    static std::unique_ptr<FileBase> Open(fujiHost* host, std::string const& filename, File::Mode mode = File::Mode::OREAD)
    {
        return std::make_unique<FileBase>(host, filename, mode);
    }

    static std::unique_ptr<FileBase> OpenBinary(fujiHost* host, std::string const& filename, File::Mode mode = File::Mode::OREAD)
    {
        return std::make_unique<FileBinary>(host, filename, mode);
    }
    //static unique_ptr<FileBase> OpenText(fujiHost* host, std::string const& filename) { return std::make_unique<FileText>(host, filename); }
};


#endif //FILEWRAPPER_H
