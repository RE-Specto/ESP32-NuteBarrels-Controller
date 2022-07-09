//#pragma once
#ifndef FILE_SYST
#define FILE_SYST
#include <Arduino.h>
#include <SD.h>
#include "SPIFFS.h"

class StorageClass
{
private:
    bool _isSD = false;
    bool _isSaving = false;
public:
    FS *disk;
    void begin();
    bool isSD();
    void useSD(bool sdcard);
    bool Load(const char *fname, byte *stru_p, uint16_t len);
    bool Save(const char *fname, byte *stru_p, uint16_t len);
    byte Backup();
    byte Restore();
    void LoadStructs();
    void SaveStructs();
};

extern StorageClass Filesys;
#endif