#include "ntag.h"
#include "Wire.h"
#ifdef ARDUINO_STM_NUCLEO_F103RB
//SCL = SCL/D15
//SDA = SDA/D14
HardWire HWire(1, I2C_REMAP);// | I2C_BUS_RESET); // I2c1
#else
#define HWire Wire
#endif


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

bool Ntag::getUid(byte *uid, unsigned int uidLength)
{
    byte data[UID_LENGTH];
    if(!readBlockTwoByteAddress(0x1009, data, UID_LENGTH))
    {
        return false;
    }
    memcpy(uid, data, UID_LENGTH < uidLength ? UID_LENGTH : uidLength);
    return true;
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
    return write(SRAM, address+SRAM_BASE_ADDR, pdata, length);
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
    Serial.println();
    return writeMod(address, pdata, length);
}


void Ntag::releaseI2c()
{
    //reset I2C_LOCKED bit
    writeRegister(NS_REG,0x40,0);
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
        Serial.println("not hitting block start in write");
        Serial.println("reading at address 0x" + String(blockNr*4));
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
        Serial.println("writeMod: writeLength: " + String(writeLength));
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

    // make offset explicit
    byte offset  = byteAddress % NTAG_BLOCK_SIZE;
    byte blockNr = byteAddress / NTAG_BLOCK_SIZE;

/*     Serial.println("readMod: byteAddress = " + String(byteAddress));
    Serial.println("readMod: blockNr     = " + String(blockNr));
    Serial.println("readMod: offset      = " + String(offset));
    Serial.println("readMod: length      = " + String(length)); */

    // always read the first block/page containing the byte address
    if (!readBlockTwoByteAddress(blockNr, readbuffer, NTAG_BLOCK_SIZE))
    {
        Serial.println("readMod: failed first block read");
        return false;
    }

    // number of useful bytes from first block only
    readLength = NTAG_BLOCK_SIZE - offset;
    if (readLength > length)
    {
        readLength = length;
    }

    Serial.println("readMod: first copy length = " + String(readLength));

    memcpy(wptr, readbuffer + offset, readLength);

    // advance write pointer immediately after first copy
    wptr += readLength;

    Serial.println("readMod: bytes copied after first block = " + String(wptr - pdata));

    // loop continues with next block only if bytes remain
    for (byte i = blockNr + 1; wptr < pdata + length; i++)
    {
        // explicit remaining byte count
        byte remaining = (pdata + length) - wptr;
        readLength = (remaining > NTAG_BLOCK_SIZE) ? NTAG_BLOCK_SIZE : remaining;

        /*
        Serial.println("readMod: next block = " + String(i));
        Serial.println("readMod: remaining  = " + String(remaining));
        Serial.println("readMod: readLength = " + String(readLength));
        */
        if (!readBlockTwoByteAddress(i, wptr, readLength))
        {
            Serial.println("readMod: failed in loop at block " + String(i));
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

    Serial.println("reading at add: " + String(memBlockAddress));

    if(!end_transmission()){
        return false;
    }

    if(HWire.requestFrom(_i2c_address, data_size)!=data_size){
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
    // Serial.println("writeBlockTwoByteAddress: address " + String(memBlockAddress));
    HWire.beginTransmission(_i2c_address);
    HWire.write(highByte(memBlockAddress));
    HWire.write(lowByte(memBlockAddress));
    // Serial.println("writeBlockTwoByteAddress: address high byte " + String(highByte(memBlockAddress)));
    // Serial.println("writeBlockTwoByteAddress: address low byte " + String(lowByte(memBlockAddress)));
    for (int i=0; i<NTAG_BLOCK_SIZE; i++)
    {
    // Serial.println("writeBlockTwoByteAddress: data " + String(p_data[i]));
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
