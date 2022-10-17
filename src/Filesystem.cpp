//#include "main.h"
#include "Filesystem.h"

// #include "Expanders.h"
#include "SystemState.h"
#include "Barrels.h"
#include "FlowSensor.h"
#include "PresureSensor.h"
// #include "WebServer.h"
// #include "Filesystem.h"
#include "Modem.h"
// #include "NTPClient.h"
// #include "FMSD.h"
#include "globals.h"

/*-------- Filesystem Begin ----------*/
// SD Card and SPIFFS
void StorageClass::begin()
{
    disk = &SPIFFS; // default
    byte retry = 3; // 3 times for SD
    while (retry)
    {
        if (SD.begin())
        {
            #ifdef DEBUG_SD
            LOG.println(F("SD card OK"));
            #endif
            disk = &SD;
            _isSD = true;
            break;
        }
        else
        {
            LOG.println(F("Error: Failed to initialize SD card"));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            retry--;
        }
    }
    if (!_isSD)
        SendSMS("SD Card Error - please check");
    retry = 2; // 2 times for SPIFFS
    while (retry)
    {
        if (SPIFFS.begin(true)) // formatOnFail enabled 
        {
            #ifdef DEBUG_SD
            LOG.println(F("SPIFFS OK"));
            #endif
            break;
        }
        else
        {
            LOG.println(F("Error: Failed to initialize SPIFFS"));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            retry--;
        }
    }
}

bool StorageClass::isSD()
{
    return _isSD;
}

// loading Structs from files
// filename, struct pointer, struct lenght "sizeof(myStruct)"
bool StorageClass::Load(const char *fname, byte *stru_p, uint16_t len)
{
    uint16_t count = 0;
    LOG.printf("Loading %s\t", fname);
    File file = disk->open(fname, "r");
    if (!file)
    {
        Serial.print(F("unable to open file\r\n\r\n"));
        return false;
    }
    for (; count < len; count++)
        if (file.available())
            *(stru_p + count) = file.read();
    #ifdef DEBUG_SD
    Serial.printf("%s\r\n%u out of %u Bytes read. filesize %satch.\r\n", count == len ? "successfully" : "failed", count, len, len == file.size() ? "M" : "Mism");
    #else
    Serial.println();
    #endif
    file.close();
    return count == len;
}

// use sdcard or spiffs
void StorageClass::useSD(bool sdcard)
{
    if (sdcard)
    {
        disk = &SD;
        _isSD = true;
    }
    else
    {
        disk = &SPIFFS;
        _isSD = false;
    }
}

// save structs to files
bool StorageClass::Save(const char *fname, byte *stru_p, uint16_t len)
{
    uint16_t count = 0;
    LOG.printf("Saving %s\t", fname);
    File file = disk->open(fname, "w"); // creates the file of not exist
    //file.setTimeCallback(timeCallback);
    if (!file)
    {
        Serial.print(F("unable to open file\r\n\r\n"));
        return false;
    }
    count = file.write(stru_p, len); // save Logic
    #ifdef DEBUG_SD
    Serial.printf("%s\r\n%u out of %u Bytes writen. filesize %satch.\r\n", count == len ? "successfully" : "failed", count, len, len == file.size() ? "M" : "Mism");
    #else
    Serial.println();
    #endif
    file.close();
    return count == len;
}

// backup overwrites all files on SPIFFS
byte StorageClass::Backup()
{
    LOG.println(F("Backing up all files from SD to SPIFFS"));
    File dir = SD.open("/");
    File file = dir.openNextFile();
    size_t len = 0; // file chunk lenght at the buffer
    byte counter = 0;
    while (file)
    {
        static byte buf[512];
        if (!file.size())
        {
            LOG.printf("skipping empty file %s\r\n", file.name());
        }
        else
        {
            // was unable to detect SD disconnection - file was true, name and size returned valid, only read returned zero.
            if (!file.read(buf, 512))
            {
                LOG.println(F("[E] SD not exist!!"));
                break;
            }             // protect against endless loop on SD error
            file.seek(0); // start from start
            File destFile = SPIFFS.open(file.name(), FILE_WRITE);
            while (file.available())
            {
                len = file.read(buf, 512);
                //stream.readBytes(buffer, length)
                destFile.write(buf, len);
                LOG.printf("copying %s from SD to SPIFFS %u bytes copied\r\n", destFile.name(), len);
            }
            destFile.close();
            counter++;
        }
        file.close();
        file = dir.openNextFile();
    }
    dir.close();
    LOG.printf("Backup finished. %u files copied\r\n", counter);
    return counter;
}

// restore only files missing on SD card. do not override existing files
byte StorageClass::Restore()
{
    #ifdef DEBUG_SD
    LOG.println(F("Restoring missing files from SPIFFS to SD"));
    #endif
    File dir = SPIFFS.open("/");
    File file = dir.openNextFile();
    size_t len = 0; // file chunk lenght at the buffer
    byte counter = 0;
    while (file)
    {
        //file.name() file.size());
        if (SD.exists(file.name()))
        {
            #ifdef DEBUG_SD
            LOG.printf("file %s already exist on SD\r\n", file.name());
            #endif
        }
        else if (!file.size())
        {
            #ifdef DEBUG_SD
            LOG.printf("skipping empty file %s\r\n", file.name());
            #endif
        }
        else
        {
            File destFile = SD.open(file.name(), FILE_WRITE);
            LOG.println(file.name());
            if (destFile)
            {
                static byte buf[512];
                //memset(buf, 0, 512); // zerofill the buffer
                while (file.available())
                {
                    len = file.read(buf, 512);
                    destFile.write(buf, len);
                    LOG.printf("copying %s from SPIFFS to SD %u bytes copied\r\n", destFile.name(), len);
                }
            }
            else
            {
                LOG.println(F("Error writing to SD"));
            }
            destFile.close();
            if (file.size() != destFile.size())
            {
                LOG.printf("[E] file %s size mismatch!\r\n", file.name());
            }
            else
                counter++;
        }
        file.close();
        file = dir.openNextFile();
    }
    dir.close();
    #ifdef DEBUG_SD
    LOG.printf("Restore finished. %u files copied\r\n", counter);
    #endif
    return counter;
}


void StorageClass::LoadStructs()
{
    if (_isSD)
        Restore(); // in case something is missing in the SD
    LOG.printf("Loading system from %s\r\n", _isSD ? "SD" : "SPIFFS");
    if (disk->exists("/SysState.bin"))
        State.LoadSD();
    else
    {
        LOG.println("/SysState.bin\tnot exist");
    }

    if (disk->exists("/Pressure.bin"))
        Pressure.LoadSD();
    else
    {
        LOG.println("/Pressure.bin\tnot exist");
    }

    if (disk->exists("/Flow.bin"))
        Flow.LoadSD();
    else
    {
        LOG.println("/Flow.bin\tnot exist");
    }

    if (disk->exists("/Barrels.bin"))
        Barrels.LoadSD();
    else
    {
        LOG.println("/Barrels.bin\tnot exist");
    }
}

// reimplement later to save on demand only what
// this one will be used on system stop? save all...?
// should run on a separate thread? or ommit the waits for no flow?
// can also stop all relays (if not already stopped?) Expanders.Protect
//and undo protect when finished.
void StorageClass::SaveStructs()
{
    LOG.println(F("save-all trigerred"));
    if (!_isSaving)
    { // prevent concurrent saving
        _isSaving = true;
        if (!_isSD)
        {
            LOG.println(F("SD card not ready - trying to restart"));
            SD.end();
            if (SD.begin())
            {
                LOG.println(F("SD card OK"));
                disk = &SD;
                _isSD = true;
            }
            else
            {
                LOG.println(F("SD failed. resorting to SPIFFS"));
                SendSMS("SD card unable to save");
            }
        }
        LOG.println(F("waiting for no flow.."));
        while (Flow.Get(FRESHWATER))
            ;
        // use 1 sensor for small system
        // while (Flow.Get(NUTRIENTS))
        //     ;
        LOG.println(F("no flow ok - saving..."));
        State.SaveSD();

        // reimplement to save ondemand?
        Barrels.SaveSD();
        Pressure.SaveSD();
        Flow.SaveSD();
        _isSaving = false;
        LOG.println(F("all save finished."));
    }
}

/*-------- Filesystem END ----------*/
StorageClass Filesys;