#ifndef NTAG_H
#define NTAG_H

#include "Arduino.h"
#include <Bounce2.h>

class Ntag
{
public:
    typedef enum{
        NTAG_I2C_1K,
        NTAG_I2C_2K
    }DEVICE_TYPE;
    typedef enum{
        NC_REG,
        LAST_NDEF_BLOCK,
        SRAM_MIRROR_BLOCK,
        WDT_LS,
        WDT_MS,
        I2C_CLOCK_STR,
        NS_REG
    }REGISTER_NR;
    Ntag(DEVICE_TYPE dt, byte i2c_address = DEFAULT_I2C_ADDRESS);
    void detectI2cDevices();//Comes in handy when you accidentally changed the I²C address of the NTAG.
    bool begin();
    bool getUid(byte *uid, unsigned int uidLength);
    byte getHarvestingLength();
    bool getEnergyHarvestingStatus(byte *eHarvest, unsigned int ehconfigLength);
    bool getCapabilityContainer(byte* container);
    byte getUidLength();
    bool isRfBusy();
    bool isReaderPresent();
    bool setSramMirrorRf(bool bEnable, byte mirrorBaseBlockNr);
    bool setFd_ReaderHandshake();
    //Address=address of the byte, not address of the 16byte block
    bool readEeprom(word address, byte* pdata, byte length);//starts at address 0
    bool readEepromMod(uint16_t address, byte *pdata, byte length);//starts at address 0
    //Address=address of the byte, not address of the 16byte block
    bool writeEeprom(word address, byte* pdata, byte length);//starts at address 0
    bool writeEepromMod(uint16_t address, byte* pdata, byte length);//starts at address 0
    bool readSram(word address, byte* pdata, byte length);//starts at address 0
    bool writeSram(word address, byte* pdata, byte length);//starts at address 0
    bool readRegister(REGISTER_NR regAddr, byte &value);
    bool readRegisterMod(uint16_t blockAddress, byte regAddress, byte *registerData, byte dataSize);
    bool writeRegisterMod(uint16_t blockAddress, byte regAddress, byte mask, byte registerData);
    bool writeRegister(REGISTER_NR regAddr, byte mask, byte regdat);
    bool setLastNdefBlock();
    bool lockEepromToI2c();
    bool unlockEeprom();
    void releaseI2c();
    bool enableNfc();
    bool disableNfc();
    bool enableSram();
    bool setEnergyHarvesting();

private:
    typedef enum{
        CONFIG=0x1,//BLOCK0 (putting this in a separate block type, because errors here can "brick" the device.)
        USERMEM=0x2,//EEPROM
        REGISTER=0x4,//Settings registers
        SRAM=0x8
    }BLOCK_TYPE;
    static const byte UID_LENGTH=2;
    static const byte EH_CONFIG_LENGTH=1;
    static const byte DEFAULT_I2C_ADDRESS=0x54;
    static const byte NTAG_BLOCK_SIZE=4;
    static const word EEPROM_BASE_ADDR=(0x1<<4);
    static const word SRAM_BASE_ADDR=(0xF8<<4);
    //Address=address of the byte, not address of the 16byte block
    bool write(BLOCK_TYPE bt, word byteAddress, byte* pdata, byte length);
    bool writeMod(uint16_t byteAddress, byte* pdata, byte length);
    //Address=address of the byte, not address of the 16byte block
    bool read(BLOCK_TYPE bt, word byteAddress, byte* pdata,  byte length);
    bool readMod(uint16_t byteAddress, byte* pdata,  byte length);
    bool readBlock(BLOCK_TYPE bt, byte memBlockAddress, byte *p_data, byte data_size);
    bool readBlockTwoByteAddress(uint16_t memBlockAddress, byte *p_data, byte data_size);
    bool writeBlock(BLOCK_TYPE bt, byte memBlockAddress, byte *p_data);
    bool writeBlockAddress(BLOCK_TYPE dt, byte addr);
    bool writeBlockTwoByteAddress(uint16_t memBlockAddress, byte *p_data);
    bool end_transmission(void);
    bool isAddressValid(BLOCK_TYPE dt, byte blocknr);
    bool setLastNdefBlock(byte memBlockAddress);
    byte _i2c_address;
    DEVICE_TYPE _dt;
    byte _lastMemBlockWritten;
    byte _mirrorBaseBlockNr;
    Bounce _debouncer;
    unsigned long _rfBusyStartTime;
    bool _triggered;
};

#endif // NTAG_H
