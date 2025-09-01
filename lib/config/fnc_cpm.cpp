#include "fnConfig.h"
#include "utils.h"
#include <cstring>

// Saves CPM DIS/ENabled flag
void fnConfig::store_cpm_enabled(bool cpm_enabled)
{
    if (_cpm.cpm_enabled == cpm_enabled)
        return;

    _cpm.cpm_enabled = cpm_enabled;
    _dirty = true;
}

// Saves CPM Command Control Processor Filename
void fnConfig::store_ccp_filename(const std::string &filename)
{
    if (_cpm.ccp == filename)
        return;

    _cpm.ccp = filename;
    _dirty = true;
}

void fnConfig::_read_section_cpm(std::stringstream &ss)
{
    std::string line;

    // Read lines until one starts with '[' which indicates a new section
    while (_read_line(ss, line, '[') >= 0)
    {
        std::string name;
        std::string value;
        if (_split_name_value(line, name, value))
        {
            if (strcasecmp(name.c_str(), "ccp") == 0)
                _cpm.ccp = value;
            else if (strcasecmp(name.c_str(), "cpm_enabled") == 0)
                _cpm.cpm_enabled = util_string_value_is_true(value);
        }
    }
}
