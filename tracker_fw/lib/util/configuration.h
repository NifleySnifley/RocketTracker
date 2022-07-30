#include <pb_common.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include <EEPROM.h>

#include "configuration.pb.h"

#define EEPROM_CONFIGFLAG_OFFS 0
#define EEPROM_CONFIGLEN_OFFS EEPROM_CONFIGFLAG_OFFS+1
#define EEPROM_CONFIGURATION_OFFS EEPROM_CONFIGLEN_OFFS + sizeof(size_t)

#define EEPROM_CONFIGURATION_ABSENT 0
#define EEPROM_CONFIGURATION_VALID 1

uint8_t configbuf[E2END - 1 - EEPROM_CONFIGURATION_OFFS];

/// Generates the default configuration
const Configuration CONFIG_DEFAULT = Configuration{
    .id = 1
};

/// Writes config to EEPROM
bool writeConfig(const Configuration& cfg) {
    pb_ostream_t stream = pb_ostream_from_buffer(configbuf, sizeof(configbuf));
    bool res = pb_encode(&stream, Configuration_fields, &cfg);

    if (res) {
        uint8_t* lenbytes = (uint8_t*)&stream.bytes_written;
        for (int i = 0; i < sizeof(size_t); ++i)
            EEPROM[EEPROM_CONFIGLEN_OFFS + i] = lenbytes[i];

        // Write to EEPROM
        for (int i = 0; i < sizeof(configbuf); ++i)
            EEPROM[EEPROM_CONFIGURATION_OFFS + i] = configbuf[i];

        // Set the flag indicating that there is a valid configuration on the EEPROM
        EEPROM[EEPROM_CONFIGFLAG_OFFS] = EEPROM_CONFIGURATION_VALID;
    } else {
        return res;
    }
}

/// Deletes the current configuration stored in EEPROM
/// Actually, it doesn't erase the config, but it sets a flag which results in the configuration being overriden with CONFIG_DEFAULT next time it is read.
void cleanConfig() {
    EEPROM[EEPROM_CONFIGFLAG_OFFS] = EEPROM_CONFIGURATION_ABSENT;
}

/// Reads config from EEPROM or loads the default (specified by CONFIG_DEFAULT)
bool readConfig(Configuration* cfg) {
    // Write default configuration if there is no configuration present in the EEPROM space dedicated to the configuration file
    if (EEPROM[EEPROM_CONFIGFLAG_OFFS] != EEPROM_CONFIGURATION_VALID) {
        writeConfig(CONFIG_DEFAULT);
    }

    size_t configlen = 0;
    for (int i = 0; i < sizeof(size_t); ++i)
        ((uint8_t*)&configlen)[i] = EEPROM[EEPROM_CONFIGLEN_OFFS + i];

    // Read from EEPROM
    for (int i = 0; i < configlen; i++)
        configbuf[i] = EEPROM[EEPROM_CONFIGURATION_OFFS + i];

    pb_istream_t configdata = pb_istream_from_buffer(configbuf, configlen);
    return pb_decode(&configdata, Configuration_fields, cfg);
}