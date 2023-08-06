//
// Created by justin on 06/08/23.
//

#ifndef FILEWRAPPER_H
#define FILEWRAPPER_H

#include <memory>
#include "FileBase.h"
#include "fujiHost.h"

struct FileWrapper{
    static std::unique_ptr<FileBase> Open(fujiHost* host, std::string const& filename) { return std::make_unique<FileBase>(host, filename); }
    //static unique_ptr<FileBase> OpenText(fujiHost* host, std::string const& filename) { return std::make_unique<FileText>(host, filename); }
};


#endif //FILEWRAPPER_H
