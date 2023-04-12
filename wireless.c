
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "gpio.h"
#include "wireless.h"
#include "timer_wireless.h"
#include "spi1.h"
#include "eeprom.h"


bool nrfSyncEnabled = false;
bool nrfJoinEnabled = false;
bool nrfJoinEnabled_BR = false;

uint8_t syncMsg[6] = {0x55,0xAA,0x55,0xAA,0x55};
uint8_t startCode[2] = {0xFE, 0xFE};
uint8_t myMac[6] = {75,78,1,2,3,4};

uint8_t slotNo  = 0;
uint16_t syncFrameCount = 0;
uint16_t packetLength =0;

uint8_t allocatedDevNum =0;

uint8_t Rxpacket[DATA_MAX_SIZE] = {0};
uint16_t Rx_index =0;

bool isSync = false;
bool isMyPacket = false;
bool msgAcked = false;
bool isDevPacket = false;
bool isBridgePacket = false;
bool isJoinPacket = false;

bool downlinkSlotStart_br  = false; // Wait for the Downlink slot start
bool fastackSlotStart_br = false;
bool acessSlotStart_br = false;
bool uplinkSlot_br = false;
bool sendJoinResponse_BR = false;

bool uplinkSlot_dev = false;
bool acessSlotStart_dev = false;

bool txTimeStatus = false; // Its okay for you to transmit
bool appSendFlag = false; // Flag for the application layer to send the data. Make it true to send data.
//----------------------------------------------------
//Debug
//----------------------------------------------------

uint8_t fifoStatus = 0;

//----------------------------------------------------
// Initialize hardware
//----------------------------------------------------

void nrf24l0Init()
{

    // Assume that the clocks are initialized already

    enablePort(PORTF);
    selectPinPushPullOutput(SYNC_LED); //RED LED
    selectPinPushPullOutput(JOIN_LED); //BLUE LED
    selectPinDigitalInput(JOIN_BUTTON);
    enablePinPullup(JOIN_BUTTON);



    enablePort(PORTB); // used for Chip enable
    selectPinPushPullOutput(CE_GPIO);

    setPinValue(CE_GPIO,1); // init always in Rx mode so that isnrfDataAvailable checks for available data
    waitMicrosecond(TX_RX_DELAY);

    // SPI1 Initialization
    initSpi1(USE_SSI_RX);
    setSpi1BaudRate(1e6, 40e6); // 1Mhz baud
    setSpi1Mode(0,0);
    selectPinPushPullOutput(SSI1FSS);

    //Initialize nrf chip
    nrf24l0ChipInit();

    initTimer_ms();

    setPinValue(JOIN_LED, 1); // Init Flash
    waitMicrosecond(100000);
    setPinValue(JOIN_LED, 0);
    waitMicrosecond(100000);

}

void nrf24l0ChipInit()
{
    uint8_t data =0;

    //configure the module
    setPinValue(CE_GPIO, 0);
    disableNrfCs();

    waitMicrosecond(5000);

    data = 0x00;                    // Interrupts off,no CRC, PWR_UP, Def Rx mode
    writeNrfReg(W_REGISTER|CONFIG,data );

    data = 0x00;                    // Turn off retransmission   /*Check for using retransmission later*/
    writeNrfReg(W_REGISTER|SETUP_RETR,data );

    data = 0x01;                    // RF data rate to 1Mbps / 1 Mhz and power is -12 dbm with 7.5 mA consumption (medium power consumption)
    writeNrfReg(W_REGISTER|RF_SETUP,data );

    data = 0x05;                                // enable dynamic payloads, enable Tx_payload, NO_ACK
    writeNrfReg(W_REGISTER|FEATURE,data );

    data = 0x3F;                                // Configure dynamic payload lengths for all data pipes
    writeNrfReg(W_REGISTER|DYNPD,data );

    data = 0x70;
    writeNrfReg(W_REGISTER|STATUS, data);

    data = 0x01;                    // Set RF channel to be 2400 + 1(data) mhz
    writeNrfReg(W_REGISTER|RF_CH,data );

    //data = 0x03;                    // Interrupts off,no CRC, PWR_UP, Def Rx mode
    //writeNrfReg(W_REGISTER|CONFIG,data );

    //waitMicrosecond(1500); // Wait 1.5 ms for PWRUP

    data = 0x00;                    // Turn off auto ACKs
    writeNrfReg(W_REGISTER|EN_AA,data );

/*// Turn on default Rx pipes
    data = 0x03;
    writeNrfReg(W_REGISTER|EN_RXADDR,data );


    data = 0x01;                    // 3byte Rx/Tx address width
    writeNrfReg(W_REGISTER|SETUP_AW,data );
*/






/*
    uint32_t pipeaddr = 0xAFAFAF;                    // Address for rx pipes

    enableNrfCs();
    writeSpi1Data(W_REGISTER|RX_ADDR_P0);
    readSpi1Data();
    writeSpi1Data(pipeaddr);
    readSpi1Data();
    disableNrfCs();

    enableNrfCs();
    writeSpi1Data(W_REGISTER|RX_ADDR_P0 +1);
    readSpi1Data();
    writeSpi1Data(pipeaddr);
    readSpi1Data();
    disableNrfCs();

    data = 0xAF;
    writeNrfReg(W_REGISTER|RX_ADDR_P0 +2,data );
    writeNrfReg(W_REGISTER|RX_ADDR_P0 +3,data );
    writeNrfReg(W_REGISTER|RX_ADDR_P0 +4,data );
    writeNrfReg(W_REGISTER|RX_ADDR_P0 +5,data );

     // Set Tx address
    enableNrfCs();
    writeSpi1Data(W_REGISTER|TX_ADDR);
    readSpi1Data();
    writeSpi1Data(pipeaddr);
    readSpi1Data();
    disableNrfCs();

    */



    enableNrfCs();                              // Flush Tx FIFO after sending a SYNC
    writeSpi1Data(W_REGISTER|FLUSH_TX);
    readSpi1Data();
    disableNrfCs();

    enableNrfCs();                              // Flush Tx FIFO after sending a SYNC
    writeSpi1Data(W_REGISTER|FLUSH_RX);
    readSpi1Data();
    disableNrfCs();

    nrf24l0PowerUp();

}
//----------------------------------------------------
// Timer functions
//----------------------------------------------------


void syncRxDevSlot(){
// Check if Sync was received by the device and wait for respective slots
    if(nrfJoinEnabled){
        startOneshotTimer_ms(joinAccessSlot_DEV, ACCESS_SLOT);
    }
    else{
        uint32_t myUplinkSlot = getMySlot(eepromGetDevInfo_DEV()); // Get the time you have to wait for your slot
        startOneshotTimer_ms(uplinkSlot_DEV, myUplinkSlot);
    }
    /*reset all the checks/data */
    isSync = false;
    uplinkSlot_dev = false;     // turned on by timer in its slot
    acessSlotStart_dev = false; // Turned on by timer if join enabled in device
    txTimeStatus = false; // Turned on by one shot timers
}
void uplinkSlot_DEV(){
    txTimeStatus = true;
    acessSlotStart_dev = false;
    uplinkSlot_dev = true; // Turn off after transmission
}

void joinAccessSlot_DEV(){
    txTimeStatus = true;
    acessSlotStart_dev = true; // turn off after transmisstion
}

void syncSlot_BR()
{
    nrfSyncEnabled = true;
    togglePinValue(SYNC_LED); // Toggle SYNC for each sync sent
}
void downlinkSlot_BR(){
    txTimeStatus = true;
}
void fastackSlot_BR(){
    txTimeStatus = true;
    downlinkSlotStart_br = false;
}
void joinAccessSlot_BR(){
    txTimeStatus = true;
    fastackSlotStart_br = false;
}
void uplinkSlot_BR(){
    acessSlotStart_br = false;
    uplinkSlot_br = false; // since uplink is not being used by bridge
}

void TimerHandler_BR()
{
    if(downlinkSlotStart_br){ // This will become true when Sync is transmitted
        startOneshotTimer_ms(downlinkSlot_BR, DL_SLOT);
    }
    if(fastackSlotStart_br){ // This will become true when DL has been transmitted
        startOneshotTimer_ms(fastackSlot_BR, FACK_SLOT);
    }
    if(acessSlotStart_br){  // This will become true when fastack has been transmitted
        startOneshotTimer_ms(joinAccessSlot_BR, ACCESS_SLOT);
    }
    if(uplinkSlot_br) {     // This will become true when acessSlotStart_br has been transmitted
    startOneshotTimer_ms(uplinkSlot_BR, UL0_SLOT);
    }
}


//----------------------------------------------------
// SPI bus functions
//----------------------------------------------------

void enableNrfCs(void)
{
    setPinValue(SSI1FSS, 0);
    _delay_cycles(4);                    // allow line to settle
}

void disableNrfCs(void)
{
    setPinValue(SSI1FSS, 1);
}

void writeNrfReg(uint8_t reg, uint8_t data)
{
    enableNrfCs();
    writeSpi1Data(reg);
    readSpi1Data();
    writeSpi1Data(data);
    readSpi1Data();
    disableNrfCs();
}

void writeNrfData(uint8_t data)
{
  //  enableNrfCs();
    writeSpi1Data(data);
    readSpi1Data();
  //  disableNrfCs();
}

void readNrfReg(uint8_t reg, uint8_t *data)
{
    enableNrfCs();
    writeSpi1Data(reg);
    readSpi1Data();
    writeSpi1Data(0);
    *data = readSpi1Data();
    disableNrfCs();
}

void readNrfData(uint8_t *data)
{
   // enableNrfCs();
    writeSpi1Data(0);
    *data = readSpi1Data();
   // disableNrfCs();
}

//----------------------------------------------------
// Timing and Slot management functions
//----------------------------------------------------
void enableSync_BR()
{
    nrfSyncEnabled = true;
}

void enableJoin_DEV()
{
    if(!getPinValue(JOIN_BUTTON)){ // wait for button press
    nrfJoinEnabled = true;
    setPinValue(JOIN_LED,1);
    }
}

void enableJoin_BR()
{
    if(!getPinValue(JOIN_BUTTON)){ // wait for button press
        nrfJoinEnabled_BR = true;
    }
}

//----------------------------------------------------
// Tx/ Rx functions
//----------------------------------------------------

void nrf24l0PulseCE(){

    setPinValue(CE_GPIO,1);
    waitMicrosecond(15);
    setPinValue(CE_GPIO,0);

}

void putNrf24l0DataPacket(uint8_t *data, uint16_t size)
{
    uint8_t j =0;
    int i = 0;

    // go to standby 1 mode
    setPinValue(CE_GPIO,0);
    waitMicrosecond(TX_RX_DELAY);

    // Tx mode setting
    writeNrfReg(W_REGISTER|CONFIG,0x02); // CRC disabled, PWRUP, Prim_rx =0
    enableNrfCs(); // Frame begins
    writeNrfData(W_TX_PAYLOAD_NO_ACK);

    //Write data byte by byte
    for(i = (size- 1), j =0; i >= 0;i--, j++) {

        if( (j != 0 ) && (j % 32) == 0 ) {
            nrf24l0PulseCE();
            while(true != nrf24l0TxStatus()); //for every 32 bytes high to low transition //
            writeNrfReg(W_REGISTER|STATUS,0x20);
            disableNrfCs();
            waitMicrosecond(TX_RX_DELAY); // ? add more delay ?? based on packet size
            enableNrfCs();
            writeNrfData(W_TX_PAYLOAD_NO_ACK);
        }
        writeNrfData(data[i]);
    }
    disableNrfCs(); // Frame ends

    nrf24l0PulseCE();

    waitMicrosecond(TX_RX_DELAY);

    // poll till data is transmitted
    while(true != nrf24l0TxStatus());
    writeNrfReg(W_REGISTER|STATUS,0x20); // clear the TX bit


    //writeNrfReg(W_REGISTER|CONFIG,0x03);    // go to rx mode, prim_rx=1, pwrup
    //waitMicrosecond(TX_RX_DELAY);                   // Rx setting

}

void getnrf24l01DataPacket(){ // Get 32 bytes of data at a time

    int i=0;
    uint8_t payloadlength =32;
    //CE enable to Rx mode//
    setPinValue(CE_GPIO,1);
    writeNrfReg(W_REGISTER|CONFIG,0x03); // CRC disabled, PWRUP, Prim_Rx =1
    waitMicrosecond(TX_RX_DELAY);
    readNrfReg(R_REGISTER|R_RX_PL_WID, &payloadlength); // get the payload length of Rx packet


    enableNrfCs();                      // Frame begins
    writeNrfData(R_RX_PAYLOAD);

    while(i<payloadlength){
        readNrfData(&Rxpacket[Rx_index + payloadlength -1]); // Convert from big endian to little endian
        Rx_index = (Rx_index +1)%DATA_MAX_SIZE;
        ++i;
    }
    disableNrfCs();                     // Frame ends
}

void parsenrf24l01DataPacket(){
    // Parse the 32 bytes from getRxpacket and check for Sync, Start code, Slot number, Length , Data


    uint8_t deviceNum = 0;

    if(strncmp((char*)Rxpacket, (char*)syncMsg, sizeof(syncMsg) -1) == 0){ // check sync. Last byte for frame count is omitted
        isSync = true;
    }
    else{
        if(strncmp((char*)Rxpacket, (char*)startCode, sizeof(startCode)) == 0){ // check start code
            deviceNum = eepromGetDevInfo_DEV(); //Get device number stored in EEPROm. For already stored devices
            packetLength = Rxpacket[3] << 8; // MSB ? check this
            packetLength |= Rxpacket[4];  // LSB

            if(Rxpacket[2] == 1){ // DL slot
                isDevPacket = true;
                /*Used in setting dev no in downlink slot */
            }
            else if(Rxpacket[2] == 2){ // FACK slot
               if( Rxpacket[5] == deviceNum){ // check whether myDevice got an ACK
                   msgAcked = true;
               }
            }
            else if(Rxpacket[2] == 3){ // Msg in ACCESS slot
                isJoinPacket = true;
                allocatedDevNum = eepromSetGetDevInfo_BR(&Rxpacket[5]); // Send the Mac address of the Joined device and save it in list of devices in Bridge

            }
            else if(Rxpacket[2] > 3){ // Msg in ACCESS slot
                isBridgePacket = true; // Is this packet received by the Bridge
            }

        }
    }


}

bool nrf24l0TxStatus()
{
    uint8_t data = 0;
    readNrfReg(R_REGISTER|STATUS,&data);
    return ((data & 0x20) >> 5); // bit 5 is Tx_DS data sent fifo interrupt
}

bool nrf24l0RxStatus()
{
    uint8_t data = 0;
    readNrfReg(R_REGISTER|FIFO_STATUS,&data);
    if( ((data & 0x0E) == 0x0E) || ((data & 0x0E) == 0x0C)
       )
    {
        return false;
    }
    else
    {
        return true;
    }
}

void nrf24l0PowerUp()
{
    uint8_t data;

    data = 0;
    readNrfReg(R_REGISTER|CONFIG, &data);

    if(!(data & 0x02)) {
        writeNrfReg(W_REGISTER|CONFIG, data | 0x02);
    }

    waitMicrosecond(5000); // Wait 5 ms for checking that data write is complete
}

bool isNrf24l0DataAvailable(void) // Check that data is available in Rx FIFO
{
    uint8_t data = 0;

    nrf24l0PowerUp();

    readNrfReg(R_REGISTER|CONFIG, &data);
    writeNrfReg(W_REGISTER|CONFIG, data | 0x1);

    data = 0x70;
    writeNrfReg(W_REGISTER|STATUS, data);

    setPinValue(CE_GPIO,1);

    waitMicrosecond(TX_RX_DELAY);

    if(nrf24l0RxStatus()){
        writeNrfReg(W_REGISTER|STATUS,0x40); // Clear the Rx interrupt
        return true;
    }
    else{
        return false;
    }
}

callback dataReceived(uint8_t *data, uint16_t size)
{
    uint8_t i;
    uint8_t buffer[30] = {0};

    /*data copy*/
    for(i = 0; i < size; i++) {
        buffer[i] = data[i];
    }
}


uint8_t nrf24l0GetChecksum(uint8_t *ptr, uint16_t size)
{
    uint8_t sum = 0;
    uint8_t i = 0;

    /*1 bytes checksum */
    for(i = 0; i < size; i ++)
    {
        sum += ptr[i];
    }

    return ~sum;
}

void nrf24l0TxSync()
{
    if(nrfSyncEnabled)
    {

        syncMsg[sizeof(syncMsg) -1] = syncFrameCount; // Frame count
        putNrf24l0DataPacket(syncMsg, sizeof(syncMsg));                                                         /*send sync message*/
        nrfSyncEnabled = false;
        ++syncFrameCount;
        uint32_t mySlotTemp = getMySlot(readEeprom(NO_OF_DEV_IN_BRIDGE));
        startPeriodicTimer_ms(syncSlot_BR, mySlotTemp);
        downlinkSlotStart_br = true;
       // setPinValue(SYNC_LED, 1);
    }
}

void nrf24l0TxJoinReq_DEV()
{
    uint8_t joinBuffer[10] = {0};
    uint8_t* ptr = joinBuffer;
    uint8_t slot = 3; // Accessslot
    uint8_t remlen = 1+sizeof(myMac);

    strncpy((char*)ptr,(char*)startCode,sizeof(startCode));
    ptr += sizeof(startCode);
    strncpy((char*)ptr , (char*)&slot, sizeof(slot));
    ptr += sizeof(slot);
    strncpy((char*)ptr , (char*)&remlen, sizeof(remlen));
    ptr += sizeof(remlen);
    strncpy((char*)ptr , (char*)myMac, sizeof(myMac));
    ptr += sizeof(myMac);
    *ptr += nrf24l0GetChecksum(joinBuffer,sizeof(joinBuffer));

    //send join request in access slot
    putNrf24l0DataPacket(joinBuffer, sizeof(joinBuffer));
    txTimeStatus = false;
//    nrfJoinEnabled = false;
    acessSlotStart_dev = false;
}

void nrf24l0TxJoinResp_BR()
{
    uint8_t joinBuffer[10] = {0};
    uint8_t* ptr = joinBuffer;
    uint8_t slot = 1; // DL slot
    uint8_t remlen = 1+1; //crc + dev no

    strncpy((char*)ptr,(char*)startCode,sizeof(startCode));
    ptr += sizeof(startCode);
    strncpy((char*)ptr , (char*)&slot, sizeof(slot));
    ptr += sizeof(slot);
    strncpy((char*)ptr , (char*)&remlen, sizeof(remlen));
    ptr += sizeof(remlen);
    strncpy((char*)ptr , (char*)&allocatedDevNum, sizeof(allocatedDevNum));
    ptr += sizeof(allocatedDevNum);
    *ptr += nrf24l0GetChecksum(joinBuffer,sizeof(joinBuffer));

    //send join response in access slot//
    putNrf24l0DataPacket(joinBuffer, sizeof(joinBuffer));
    txTimeStatus = false;
    nrfJoinEnabled_BR = false;
    sendJoinResponse_BR = false;

}

//void nrf24l0TxDownlink_BR()

int nrf24l0TxMsg(uint8_t* data, uint16_t size, uint8_t devno)
{
    uint8_t packet[DATA_MAX_SIZE] = {0};
    uint8_t *ptr = packet;
    uint16_t remlen = size+1; /*Increment lenght for checksum */

        if(size > (DATA_MAX_SIZE - META_DATA_SIZE))
        {
            return -1;
        }

        /*meta data*/
        strncpy((char*)ptr,(char*)startCode,sizeof(startCode));
        ptr += sizeof(startCode);
        strncpy((char*)ptr , (char*)&slotNo, sizeof(slotNo));
        ptr += sizeof(slotNo);
        strncpy((char*)ptr , (char*)&remlen, sizeof(remlen));
        ptr += sizeof(remlen);
//Use devno for sending DL packets
        //user data//
        strncpy((char*)ptr, (char*)data, size);
        ptr += size;

        //calculate checksum//
        *ptr = nrf24l0GetChecksum(packet,size + META_DATA_SIZE);

        putNrf24l0DataPacket(packet, size + META_DATA_SIZE);

        //guard timer //
        waitMicrosecond(GUARD_TIMER);

    return 0;
}



void nrf24l0RxMsg(callback fn)
{
    // Devices use this function as used in template code to keep on polling the data to check for received messages
    //**SYNC
    // Check for Sync message and start timers for the devices transmission slot
    // If a new device is Joining a timer is started to wait for transmission in ACCESS slot
    // If the bridge is transmitting (assumption that Bridge device also sends the Sync packets), Wait for Downlink and FASTACK slots to transmit data
    uint16_t size = 0;
    uint8_t Rxchecksum = 0; // Received checksum
    uint8_t devMac[6] = {0};
    uint8_t i,j = 0;


    if(isNrf24l0DataAvailable()) {
        togglePinValue(JOIN_LED);

        getnrf24l01DataPacket();
        parsenrf24l01DataPacket(); // set flags if syn message or startcode or

        if(isSync) //if sync //
        {
            syncRxDevSlot();                   // Handle device sync slots
            togglePinValue(SYNC_LED);          // Toggle SYNC for each sync received
        }

        if((isBridgePacket && Rx_index >= packetLength) ||(isDevPacket && Rx_index >= packetLength))
        {                                               // is data for me? and have I received whole packet*/
                                                        //crc check*/
            Rxchecksum = Rxpacket[Rx_index-1];
            Rxchecksum = ~Rxchecksum;
            if(Rxchecksum == nrf24l0GetChecksum(Rxpacket,packetLength))
            {
                //Devices and Bridge can pop the data out and utilize it as needed

                fn(Rxpacket, size);                     // downlink data. Callback function */
                Rx_index = 0;                           // reset for next packet Rx
                packetLength = 0;
            }
                                                        //reset the buffer. */ //Extra precaution
            for (i=0; i<sizeof(Rxpacket); i++){
                Rxpacket[i] =0;
            }
        }

        if(msgAcked) //check acks for my device //
        {
            // Received ack . Stop retransmission. Send this to Application layer to stop retransmissting messages
        }

        if(nrfJoinEnabled && isDevPacket) //join response for dev
        {
            eepromSetDevInfo_DEV(Rxpacket[5]);
            setPinValue(JOIN_LED,0);
            nrfJoinEnabled = false; // This is true from the moment dev sends a join req and receives a
        }

        if(isJoinPacket){ //
            // Send Join response to device with allocated Devicenum
            // Differentiate between join response and regular data in downlink for device
            if(nrfJoinEnabled_BR && acessSlotStart_br)
            {
                for(i = 0, j = 5; i< 6; i++, j++)
                {
                    devMac[i] = Rxpacket[j];
                }
                sendJoinResponse_BR = true;
                allocatedDevNum =  eepromSetGetDevInfo_BR(devMac); // Set mac address of bridge in eeprom
            }
        }
    }

}

uint32_t getMySlot(uint8_t devno)
{
    // default value
    if(devno == 0xFF) {
        devno = 0;
    }

    return UL0_SLOT + (((22) + GUARD_TIMER)*devno) ;

    //Handle slots for all transmissions for Devices and bridges separately
}

void eepromSetDevInfo_DEV(uint8_t deviceNum)
{
    uint8_t *ptr;
    uint8_t i;

    writeEeprom(NO_OF_DEV_IN_BRIDGE,1);                         // Number of devices is 1
    writeEeprom(DEV1_NO_START,deviceNum);                       // Device number

    ptr = (uint8_t*)DEV1_MAC_START;

    for(i = 0; i < sizeof(myMac); i++) {                                    // My Mac address
        ptr[i] = myMac[i];
    }

}

uint8_t eepromGetDevInfo_DEV()
{
    return readEeprom(DEV1_NO_START); // Only one device for Dev; Device number
}

uint8_t eepromSetGetDevInfo_BR(uint8_t *data)
{
    int i=0;
    uint8_t devNo = 0;
    uint8_t* ptr = (uint8_t*)NO_OF_DEV_IN_BRIDGE;

    uint8_t noOfDev = (readEeprom(NO_OF_DEV_IN_BRIDGE) == 0xFF) ? 0 : readEeprom(NO_OF_DEV_IN_BRIDGE);

    for(i = 0; i < noOfDev; i++) {
        ptr = (uint8_t*)DEV1_MAC_START;

        if(strncmp((char*)ptr,(char*)data,sizeof(myMac)) == 0) {
            return *--ptr;
        }
        ptr += 1 + sizeof(myMac);
    }

    if(noOfDev == 0) {
        ptr = (uint8_t*)DEV1_MAC_START;
    }

    --ptr;
    *ptr = noOfDev++;
    devNo = *ptr;
    writeEeprom(NO_OF_DEV_IN_BRIDGE,devNo); /* store total no of devices */
    ptr ++;

    /*mac*/
    for(i = 0; i < sizeof(myMac); i++) {
        ptr[i] = data[i];
    }

    return devNo;
}

/*
int main()
{
    initSystemClockTo40Mhz();
    initEeprom();
    nrf24l0Init();
    // Flush Rx for device and bridge
    uint8_t data[] = {1,2,3,4,5}; // Random data, magic number change later
    enableSync_BR();

    while(true)
    {
        // debug
        readNrfReg(R_REGISTER|FIFO_STATUS,&fifoStatus);

// Bridge Functions //
        nrf24l0TxSync();
        TimerHandler_BR();
        enableJoin_BR();

//Common functions//
        nrf24l0RxMsg(dataReceived);

        //check for slot no//
        if(txTimeStatus) // change later
        {
            if(sendJoinResponse_BR && downlinkSlotStart_br){
                nrf24l0TxJoinResp_BR();
                }
            else{
                if(appSendFlag){
                    nrf24l0TxMsg(data,5, 0); // Magic number devno change later
                    appSendFlag = false;
                }
            }
        }
    }
}
*/


int main()
{
    initSystemClockTo40Mhz();
    initEeprom();
    nrf24l0Init();
    uint8_t data[] = {1,2,3,4,5}; // Random data, magic number change later

    while(true)
        {
    //Device functions
        enableJoin_DEV();
    //Common functions
        readNrfReg(R_REGISTER|FIFO_STATUS,&fifoStatus);
        nrf24l0RxMsg(dataReceived);

        //check for slot no
        if(txTimeStatus) // change later
        {
            if(nrfJoinEnabled && acessSlotStart_dev){
                nrf24l0TxJoinReq_DEV();
                }
            else{
                if(appSendFlag){
                    nrf24l0TxMsg(data,5, 0); // Magic number devno change later
                    appSendFlag = false;
                }
            }
        }
    }
}



