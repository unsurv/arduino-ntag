#include "ntag.h"
#include "Wire.h"
#ifdef ARDUINO_STM_NUCLEO_F103RB
//SCL = SCL/D15
//SDA = SDA/D14
HardWire HWire(1, I2C_REMAP);// | I2C_BUS_RESET); // I2c1
#else
#define HWire Wire
#endif
// #define NFC_SENSE_DEBUG

Ntag::Ntag(DEVICE_TYPE dt, byte i2c_address):
    _dt(dt),
    _i2c_address(i2c_address),
    _rfBusyStartTime(0),
    _triggered(false)
{
    _debouncer = Bounce();
}

bool Ntag::begin(){
    bool bResult=true;
    HWire.begin();
#ifndef ARDUINO_SAM_DUE
    HWire.beginTransmission(_i2c_address);
    bResult=HWire.endTransmission()==0;
#else
    //Arduino Due always sends at least 2 bytes for every I²C operation.  This upsets the NTAG.
    return true;
#endif

    return bResult;
}

bool Ntag::isReaderPresent()
{
    return false;
}

void Ntag::detectI2cDevices(){
    for(byte i=0;i<0x80;i++){
        HWire.beginTransmission(i);
        if(HWire.endTransmission()==0)
        {
            Serial.print("Found I²C device on : 0x");
            Serial.println(i,HEX);
        }
    }
}

byte Ntag::getUidLength()
{
    return UID_LENGTH;
}

byte Ntag::getHarvestingLength()
{
    return EH_CONFIG_LENGTH;
}

bool Ntag::getUid(byte *uid, unsigned int uidLength)
{
    byte data[UID_LENGTH];
    if(!readBlockTwoByteAddress(0x1009, data, UID_LENGTH))
    {   
        Serial.println("uid get failed");
        return false;
    }
    memcpy(uid, data, UID_LENGTH < uidLength ? UID_LENGTH : uidLength);
    return true;
}

bool Ntag::getEnergyHarvestingStatus(byte *eHarvest, unsigned int ehconfigLength)
{
    byte data[EH_CONFIG_LENGTH];
    if(!readBlockTwoByteAddress(0x103D, data, EH_CONFIG_LENGTH))
    {
        return false;
    }
    memcpy(eHarvest, data, EH_CONFIG_LENGTH < ehconfigLength ? EH_CONFIG_LENGTH : ehconfigLength);
    
    // check bit 0 for energy harvest enable
    if (data[0] & 0x01) {
        return true;
    } else {
        return false;
    }

}



bool Ntag::getCapabilityContainer(byte* container)
{
    if (!container)
    {
        Serial.println("getCapabilityContainer: no container");
        return false;
    }

    // Type 5 CC starts in block 0
    if (!readBlockTwoByteAddress(0, container, 4))
    {
        Serial.println("getCapabilityContainer: failed at container read");
        return false;
    }

    return true;
}



bool Ntag::setFd_ReaderHandshake(){
    //return writeRegister(NC_REG, 0x3C,0x18);
    return writeRegister(NC_REG, 0x3C,0x28);
    //0x28: FD_OFF=10b, FD_ON=10b : FD constant low
    //Start of read by reader always clears the FD-pin.
    //At the end of the read by reader, the FD-pin becomes high (most of the times)
    //0x18: FD pulse high (13.9ms wide) at the beginning of the read sequence, no effect on write sequence.
    //0x14: FD_OFF=01b, FD_ON=01b : FD constant high
    //0x24: FD constant high
}

bool Ntag::isRfBusy(){
    byte regVal;
    const byte RF_LOCKED=5;
    _debouncer.update();
    //Reading this register clears the FD-pin.
    //When continuously polling this register while RF reading or writing is ongoing, high will be returned for 2ms, followed
    //by low for 9ms, then high again for 2ms then low again for 9ms and so on.
    //To get a nice clean high or low instead of spikes, a software retriggerable monostable that triggers on rfBusy will be used.
    if(!readRegister(NS_REG, regVal))
    {
        Serial.println("Can't read register.");
    }
    if(bitRead(regVal,RF_LOCKED) || _debouncer.rose())
    {
        //retrigger monostable
        _rfBusyStartTime=millis();
        _triggered=true;
        return true;
    }
    if(_triggered && millis()<_rfBusyStartTime+30)
    {
        //a zero has been read, but monostable hasn't run out yet
        return true;
    }
    return false;
}

//Mirror SRAM to EEPROM
//Remark that the SRAM mirroring is only valid for the RF-interface.
//For the I²C-interface, you still have to use blocks 0xF8 and higher to access SRAM area (see datasheet Table 6)
bool Ntag::setSramMirrorRf(bool bEnable, byte mirrorBaseBlockNr){
    _mirrorBaseBlockNr = bEnable ? mirrorBaseBlockNr : 0;
    if(!writeRegister(SRAM_MIRROR_BLOCK,0xFF,mirrorBaseBlockNr)){
        return false;
    }
    //disable pass-through mode (because it's not compatible with SRAM⁻mirroring: datasheet §11.2).
    //enable/disable SRAM memory mirror
    return writeRegister(NC_REG, 0x42, bEnable ? 0x02 : 0x00);
}

bool Ntag::readSram(word address, byte *pdata, byte length)
{
    return read(SRAM, address+SRAM_BASE_ADDR, pdata, length);
}

bool Ntag::writeSram(word address, byte *pdata, byte length)
{
    return writeMod(address, pdata, length);
}

bool Ntag::readEeprom(word address, byte *pdata, byte length)
{
    return read(USERMEM, address+EEPROM_BASE_ADDR, pdata, length);
}

bool Ntag::readEepromMod(uint16_t address, byte *pdata, byte length)
{
    return readMod(address, pdata, length);
}

bool Ntag::writeEeprom(word address, byte *pdata, byte length)
{
    return write(USERMEM, address+EEPROM_BASE_ADDR, pdata, length);
}

bool Ntag::writeEepromMod(uint16_t address, byte *pdata, byte length)
{
    return writeMod(address, pdata, length);
}


void Ntag::releaseI2c()
{
    // reset I2C_IF_LOCKED bit at byte 1; bit 1; address 0x10A9
    writeRegisterMod(0x10A0, 0x01, 0x02, 0x00);

}

bool Ntag::write(BLOCK_TYPE bt, word byteAddress, byte* pdata, byte length)
{
    byte readbuffer[NTAG_BLOCK_SIZE];
    byte writeLength;
    byte* wptr=pdata;
    byte blockNr=byteAddress/NTAG_BLOCK_SIZE;

    if(byteAddress % NTAG_BLOCK_SIZE !=0)
    {
        //start address doesn't point to start of block, so the bytes in this block that precede the address range must
        //be read.
        if(!readBlock(bt, blockNr, readbuffer, NTAG_BLOCK_SIZE))
        {
            return false;
        }
        writeLength=NTAG_BLOCK_SIZE - (byteAddress % NTAG_BLOCK_SIZE);
        if(writeLength<length)
        {
            writeLength=length;
        }
        memcpy((void*)(readbuffer + (byteAddress % NTAG_BLOCK_SIZE)), pdata, writeLength);
        if(!writeBlock(bt, blockNr, readbuffer))
        {
            return false;
        }
        wptr+=writeLength;
        blockNr++;
    }
    while(wptr < pdata+length)
    {
        writeLength=(pdata+length-wptr > NTAG_BLOCK_SIZE ? NTAG_BLOCK_SIZE : pdata+length-wptr);
        if(writeLength!=NTAG_BLOCK_SIZE){
            if(!readBlock(bt, blockNr, readbuffer, NTAG_BLOCK_SIZE))
            {
                return false;
            }
            memcpy(readbuffer, wptr, writeLength);
        }
        if(!writeBlock(bt, blockNr, writeLength==NTAG_BLOCK_SIZE ? wptr : readbuffer))
        {
            return false;
        }
        wptr+=writeLength;
        blockNr++;
    }
    _lastMemBlockWritten = --blockNr;
    return true;
}


bool Ntag::writeMod(uint16_t byteAddress, byte* pdata, byte length)
{
    byte readbuffer[NTAG_BLOCK_SIZE];
    byte writeLength;
    byte* wptr=pdata;
    byte blockNr=byteAddress/NTAG_BLOCK_SIZE;

    if(byteAddress % NTAG_BLOCK_SIZE !=0)
    {
        //start address doesn't point to start of block, so the bytes in this block that precede the address range must
        //be read.
#ifdef NFC_SENSE_DEBUG
        Serial.println("not hitting block start in write");
        Serial.println("reading at address 0x" + String(blockNr*4));
#endif
        if(!readBlockTwoByteAddress(blockNr, readbuffer, NTAG_BLOCK_SIZE))
        {
            return false;
        }
        writeLength=NTAG_BLOCK_SIZE - (byteAddress % NTAG_BLOCK_SIZE);

        memcpy((void*)(readbuffer + (byteAddress % NTAG_BLOCK_SIZE)), pdata, writeLength);
        if(!writeBlockTwoByteAddress(blockNr, readbuffer))
        {
            return false;
        }
#ifdef NFC_SENSE_DEBUG
        Serial.println("writeMod: writeLength: " + String(writeLength));
#endif
        wptr+=writeLength;
        blockNr++;
    }

    while(wptr < pdata+length)
    {
        writeLength=(pdata+length-wptr > NTAG_BLOCK_SIZE ? NTAG_BLOCK_SIZE : pdata+length-wptr);
        if(writeLength!=NTAG_BLOCK_SIZE){
            if(!readBlockTwoByteAddress(blockNr, readbuffer, NTAG_BLOCK_SIZE))
            {
                return false;
            }
            memcpy(readbuffer, wptr, writeLength);
        }
        if(!writeBlockTwoByteAddress(blockNr, writeLength==NTAG_BLOCK_SIZE ? wptr : readbuffer))
        {
            return false;
        }

        wptr+=writeLength;
        blockNr++;
    }
    _lastMemBlockWritten = --blockNr;
    return true;
}


/*
bool Ntag::readEeprom(word address, byte *pdata, byte length)
{
    return read(USERMEM, address, pdata, length);
}
*/
bool Ntag::read(BLOCK_TYPE bt, word byteAddress, byte* pdata,  byte length)
{
    byte readbuffer[NTAG_BLOCK_SIZE];
    byte readLength;
    byte* wptr=pdata;

    readLength=(byteAddress % NTAG_BLOCK_SIZE) + length;
    if(readLength<NTAG_BLOCK_SIZE)
    {
        readLength=NTAG_BLOCK_SIZE;
    }
    if(!readBlock(bt, byteAddress/NTAG_BLOCK_SIZE, readbuffer, readLength))
    {
        return false;
    }
    readLength-=byteAddress % NTAG_BLOCK_SIZE;
    memcpy(wptr,readbuffer + (byteAddress % NTAG_BLOCK_SIZE), readLength);
    wptr+=readLength;
    for(byte i=(byteAddress/NTAG_BLOCK_SIZE)+1;wptr<pdata+length;i++)
    {
        readLength=(pdata+length-wptr > NTAG_BLOCK_SIZE ? NTAG_BLOCK_SIZE : pdata+length-wptr);
        if(!readBlock(bt, i, wptr, readLength))
        {
            return false;
        }
        wptr+=readLength;
    }
    return true;
}

bool Ntag::readMod(uint16_t byteAddress, byte* pdata, byte length)
{
    byte readbuffer[NTAG_BLOCK_SIZE];
    byte readLength;
    byte* wptr = pdata;

    byte offset  = byteAddress % NTAG_BLOCK_SIZE;
    byte blockNr = byteAddress / NTAG_BLOCK_SIZE;

#ifdef NFC_SENSE_DEBUG
    Serial.println("readMod: byteAddress = " + String(byteAddress));
    Serial.println("readMod: blockNr     = " + String(blockNr));
    Serial.println("readMod: offset      = " + String(offset));
    Serial.println("readMod: length      = " + String(length));
#endif
    // always read the first block/page containing the byte address
    if (!readBlockTwoByteAddress(blockNr, readbuffer, NTAG_BLOCK_SIZE))
    {   
        #ifdef NFC_SENSE_DEBUG
            Serial.println("readMod: failed first block read");
        #endif
        
        return false;
    }

    // number of useful bytes from first block only
    readLength = NTAG_BLOCK_SIZE - offset;
    if (readLength > length)
    {
        readLength = length;
    }
#ifdef NFC_SENSE_DEBUG
    Serial.println("readMod: first copy length = " + String(readLength));
#endif
    memcpy(wptr, readbuffer + offset, readLength);

    // advance write pointer immediately after first copy
    wptr += readLength;
#ifdef NFC_SENSE_DEBUG
    Serial.println("readMod: bytes copied after first block = " + String(wptr - pdata));
#endif
    // loop continues with next block only if bytes remain
    for (byte i = blockNr + 1; wptr < pdata + length; i++)
    {
        // explicit remaining byte count
        byte remaining = (pdata + length) - wptr;
        readLength = (remaining > NTAG_BLOCK_SIZE) ? NTAG_BLOCK_SIZE : remaining;

#ifdef NFC_SENSE_DEBUG
        Serial.println("readMod: next block = " + String(i));
        Serial.println("readMod: remaining  = " + String(remaining));
        Serial.println("readMod: readLength = " + String(readLength));
#endif
        if (!readBlockTwoByteAddress(i, wptr, readLength))
        {   
            #ifdef NFC_SENSE_DEBUG
                Serial.println("readMod: failed in loop at block " + String(i));
            #endif
            
            return false;
        }

        wptr += readLength;
    }

    return true;
}

bool Ntag::readBlock(BLOCK_TYPE bt, byte memBlockAddress, byte *p_data, byte data_size)
{
    if(data_size>NTAG_BLOCK_SIZE || !writeBlockAddress(bt, memBlockAddress)){
        return false;
    }
    if(!end_transmission()){
        return false;
    }
    if(HWire.requestFrom(_i2c_address,data_size)!=data_size){
        return false;
    }
    byte i=0;
    while(HWire.available())
    {
        p_data[i++] = HWire.read();
    }
    return i==data_size;
}

bool Ntag::readBlockTwoByteAddress(uint16_t memBlockAddress, byte *p_data, byte data_size)
{
    HWire.beginTransmission(_i2c_address);
    HWire.write(highByte(memBlockAddress));
    HWire.write(lowByte(memBlockAddress));
#ifdef NFC_SENSE_DEBUG
    Serial.println("reading at add: " + String(memBlockAddress));
#endif
    if(!end_transmission()){
        #ifdef NFC_SENSE_DEBUG
            Serial.println("bus busy");
        #endif

        return false;
    }

    if(HWire.requestFrom(_i2c_address, data_size)!=data_size){
        #ifdef NFC_SENSE_DEBUG
            Serial.println("!= data size");        
        #endif
        return false;
    }
    byte i=0;
    while(HWire.available())
    {
        p_data[i++] = HWire.read();
    }
    return i==data_size;
}

bool Ntag::setLastNdefBlock()
{
    //When SRAM mirroring is used, the LAST_NDEF_BLOCK must point to USERMEM, not to SRAM
    return writeRegister(LAST_NDEF_BLOCK, 0xFF, isAddressValid(SRAM, _lastMemBlockWritten) ?
                             _lastMemBlockWritten - (SRAM_BASE_ADDR>>4) + _mirrorBaseBlockNr : _lastMemBlockWritten);
}

bool Ntag::writeBlock(BLOCK_TYPE bt, byte memBlockAddress, byte *p_data)
{
    if(!writeBlockAddress(bt, memBlockAddress)){
        return false;
    }
    for (int i=0; i<NTAG_BLOCK_SIZE; i++)
    {
	HWire.write(p_data[i]);
    }
    if(!end_transmission()){
        return false;
    }
    switch(bt){
    case CONFIG:
    case USERMEM:
        delay(5);//16 bytes (one block) written in 4.5 ms (EEPROM)
        break;
    case REGISTER:
    case SRAM:
        delayMicroseconds(500);//0.4 ms (SRAM - Pass-through mode) including all overhead
        break;
    }
    return true;
}

bool Ntag::writeBlockTwoByteAddress(uint16_t memBlockAddress, byte *p_data)
{

    HWire.beginTransmission(_i2c_address);
    HWire.write(highByte(memBlockAddress));
    HWire.write(lowByte(memBlockAddress));
#ifdef NFC_SENSE_DEBUG
    Serial.println("writeBlockTwoByteAddress: address " + String(memBlockAddress));
    Serial.println("writeBlockTwoByteAddress: address high byte " + String(highByte(memBlockAddress)));
    Serial.println("writeBlockTwoByteAddress: address low byte " + String(lowByte(memBlockAddress)));
#endif
    for (int i=0; i<NTAG_BLOCK_SIZE; i++)
    {
	HWire.write(p_data[i]);
    }

    if(!end_transmission()){
        return false;
    }

    delay(5); //16 bytes (one block) written in 4.5 ms (EEPROM)

    return true;
}

bool Ntag::readRegister(REGISTER_NR regAddr, byte& value)
{
    value=0;
    bool bRetVal=true;
    if(regAddr>6 || !writeBlockAddress(REGISTER, 0xFE)){
        return false;
    }
    HWire.write(regAddr);
    if(!end_transmission()){
        return false;
    }
    if(HWire.requestFrom(_i2c_address,(byte)1)!=1){
        return false;
    }
    value=HWire.read();
    return bRetVal;
}

bool Ntag::readRegisterMod(uint16_t blockAddress, byte regAddress, byte *registerData, byte dataSize)
{
    // Select the NTAG register block.
    HWire.beginTransmission(_i2c_address);

    HWire.write(highByte(blockAddress));     // BL_AD1
    HWire.write(lowByte(blockAddress));   // BL_AD0
    HWire.write(regAddress);


    // START + SL_AD/W + BL_AD1 + BL_AD0 + STOP
    //
    uint8_t status = HWire.endTransmission(true);

        if (status != 0) {
#ifdef NFC_SENSE_DEBUG
        Serial.print("NTAG read-register select failed, I2C status: ");
        Serial.println(status);
#endif
        return false;
    }

    // --- Read phase: read selected register byte(s) ---
    // Sends:
    // START | SL_AD+R | DATA0 ... DATAN | NACK | STOP
    uint8_t received = HWire.requestFrom(
        (uint8_t)_i2c_address,
        (uint8_t)dataSize,
        (uint8_t)true
    );

    if (received != dataSize) {
    
    #ifdef NFC_SENSE_DEBUG
        Serial.print("NTAG read-register expected ");
        Serial.print(dataSize);
        Serial.print(", received ");
        Serial.println(received);
    #endif
        return false;
    }

    for (uint8_t i = 0; i < dataSize; ++i) {
        if (!HWire.available()) {
            return false;
        }

        registerData[i] = HWire.read();

    }
    return true;
}

bool Ntag::writeRegisterMod(uint16_t blockAddress,
                            byte regAddress,
                            byte mask,
                            byte registerData)
{
    // Select the NTAG register block and register.
    HWire.beginTransmission(_i2c_address);

    HWire.write(highByte(blockAddress));  // BL_AD1: block address MSB
    HWire.write(lowByte(blockAddress));   // BL_AD0: block address LSB
    HWire.write(regAddress);              // REGA: register address
    HWire.write(mask);                    // MASK: bits to modify
    HWire.write(registerData);            // REGDATA: data to write

    // Sends:
    // START | SL_AD+W | BL_AD1 | BL_AD0 | REGA |
    // MASK | REGDATA | STOP
    uint8_t status = HWire.endTransmission(true);

    if (status != 0) {
#ifdef NFC_SENSE_DEBUG
        Serial.print("NTAG write-register failed, I2C status: ");
        Serial.println(status);
#endif
        return false;
    }

    return true;
}

bool Ntag::writeRegister(REGISTER_NR regAddr, byte mask, byte regdat)
{
    if(regAddr>7 || !writeBlockAddress(REGISTER, 0xFE)){
        return false;
    }
    HWire.write(regAddr);
    HWire.write(mask);
    HWire.write(regdat);
    return end_transmission();
}

bool Ntag::writeBlockAddress(BLOCK_TYPE dt, byte addr)
{
    if(!isAddressValid(dt, addr)){
        return false;
    }
    HWire.beginTransmission(_i2c_address);
    HWire.write(addr);
    return true;
}

bool Ntag::end_transmission(void)
{
    return HWire.endTransmission()==0;
    //I2C_LOCKED must be either reset to 0b at the end of the I2C sequence or wait until the end of the watch dog timer.
}

bool Ntag::isAddressValid(BLOCK_TYPE type, byte blocknr){
    switch(type){
    case CONFIG:
        if(blocknr!=0){
            return false;
        }
        break;
    case USERMEM:
        switch (_dt) {
        case NTAG_I2C_1K:
            if(blocknr < 1 || blocknr > 0x38){
                return false;
            }
            break;
        case NTAG_I2C_2K:
            if(blocknr < 1 || blocknr > 0x78){
                return false;
            }
            break;
        default:
            return false;
        }
        break;
    case SRAM:
        if(blocknr < 0xF8 || blocknr > 0xFB){
            return false;
        }
        break;
    case REGISTER:
        if(blocknr != 0xFE){
            return false;
        }
        break;
    default:
        return false;
    }
    return true;
}

bool Ntag::lockEepromToI2c(){
    byte lockBytes[4] = {0x00, 0x02, 0x00, 0x00};
    return writeBlockTwoByteAddress(0x10A0, lockBytes);
}

bool Ntag::unlockEeprom(){
    byte unlockBytes[4] = {0x00, 0x00, 0x00, 0x00};
    return writeBlockTwoByteAddress(0x10A0, unlockBytes);
}

/* bool Ntag::disableNfc(){
    byte regValue[4];

    if(!readRegisterMod(0x10A1, 0, regValue, 2))
    {   
        Serial.println("NFC config get failed");
        return false;
    }


    Serial.print("NFC Status is: ");
    for(byte i=0;i<4;i++)
        {
            Serial.print(regValue[i]);
            Serial.print(" ");
        }
    Serial.println();

    writeRegisterMod(
        0x10A1,  // BL_AD
        0x00,    // REGA
        0x20,    // MASK: modify only bit 5
        0x20     // REGDATA: bit 5 = 1
    );

    byte regValue2[4];

    if(!readRegisterMod(0x10A1, 0, regValue2, 2))
    {   
        Serial.println("NFC config get failed");
        return false;
    }


    Serial.print("NFC Status is: ");
    for(byte i=0;i<4;i++)
        {
            Serial.print(regValue2[i]);
            Serial.print(" ");
        }
    Serial.println();
    
    return true;
}

bool Ntag::enableNfc(){
    
    byte regValue[4];

    if(!readRegisterMod(0x10A1, 0, regValue, 2))
    {   
        Serial.println("NFC config get failed");
        return false;
    }


    Serial.print("NFC Status is: ");
    for(byte i=0;i<4;i++)
        {
            Serial.print(regValue[i], HEX);
            Serial.print(" ");
        }
    Serial.println();

    writeRegisterMod(
        0x10A1,  // BL_AD
        0x00,    // REGA
        0x20,    // MASK: modify only bit 5
        0x00     // REGDATA: bit 5 = 0
    );

    byte regValue2[4];

    if(!readRegisterMod(0x10A1, 0, regValue2, 4))
    {   
        Serial.println("NFC config get failed");
        return false;
    }


    Serial.print("NFC Status is: ");
    for(byte i=0;i<4;i++)
        {
            Serial.print(regValue2[i]);
            Serial.print(" ");
        }
    Serial.println();
    
    return true;
} */

bool Ntag::disableNfc(){


    if(!writeRegisterMod(
        0x10A0,  // BL_AD
        0x01,    // REGA
        0x02,    // MASK: modify only bit 2
        0x02     // REGDATA: bit 5 = 1
    ))
    {   
        Serial.println("NFC config get failed");
        return false;
    }
    
    return true;
}

bool Ntag::enableNfc(){
    
       if(!writeRegisterMod(
        0x10A0,  // BL_AD
        0x01,    // REGA
        0x02,    // MASK: modify only bit 2
        0x00     // REGDATA: bit 5 = 1
    ))
    {   
        Serial.println("NFC config get failed");
        return false;
    }
    
    return true;
    
}

bool Ntag::enableSram(){
    
    byte configValue[4];
    // 0x00 0x02 0x0F 0x00 default values
    configValue[0] = 0x00;
    // 10000110 for sram enable, sram mirror mode, auto enable arbiter when energy harvesting
    // configValue2[1] = 0x86;
    // 00000010
    // configValue2[1] = 0x02;
    // 00000110 for sram enable, sram mirror mode
    // configValue2[1] = 0x06;
    // 10000000 for auto enable normal arbiter
    configValue[1] = 0x80;

    // works from time to time
    // 00000000 for auto enable normal arbiter
    // configValue2[1] = 0x00;
    
    configValue[2] = 0x0F;
    configValue[3] = 0x00;
    
    if(!writeBlockTwoByteAddress(0x1037, configValue))
    {   
        Serial.println("NFC config set failed");
        return false;
    }

    return true;

}

bool Ntag::setEnergyHarvesting(){
    
    byte configValue[4];
    // 45 0 15 0 old values
    configValue[0] = 0x2D;
    // 00110101 to disable vout check
    // configValue2[0] = 0x4D;
    // 00110101 for >2.7 mA and power check enabled 
    // configValue2[0] = 0x35;
    configValue[1] = 0x00; 
    configValue[2] = 0x0F;
    configValue[3] = 0x00;

    
    if(!writeBlockTwoByteAddress(0x103D, configValue))
    {   
        Serial.println("NFC set EH set failed");
        return false;
    }

    return true;
}
