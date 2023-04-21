
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "gpio.h"
#include "timer_wireless.h"
#include "spi1.h"
#include "eeprom.h"
#include "uart0.h"
#include "wireless.h"


bool nrfSyncEnabled = false;
bool nrfJoinEnabled = false;
bool nrfJoinEnabled_BR = false;

uint8_t syncMsg[7] = {0x55,0xAA,0x55,0xAA,0x55};
uint8_t startCode[2] = {0xFE, 0xFE};
uint8_t myMac[6] = {75,78,1,2,3,4};

uint8_t slotNo  = 0;
uint16_t syncFrameCount = 0;
uint16_t packetLength =0;

uint8_t allocatedDevNum =0;

uint8_t Rxpacket[DATA_MAX_SIZE] = {0};
uint16_t Rx_index =0; // read index
uint16_t Rx_wrIndex =0; // write index
uint8_t payloadlength =32;


bool isSync = false;
bool isMyPacket = false;
bool msgAcked = false;
bool isDevPacket = false;
bool isBridgePacket = false;
bool isJoinPacket = false;

bool downlinkSlotStart_br  = false; // Wait for the Downlink slotNo start
bool fastackSlotStart_br = false;
bool acessSlotStart_br = false;
bool uplinkSlot_br = false;
bool sendJoinResponse_BR = false;

bool timersStarted_br = false;

bool acessSlotStart_dev = false;
bool uplinkSlot_dev = false;
bool isJoinResp_dev = false;

bool txTimeStatus = false; // Its okay for you to transmit
bool appSendFlag = false; // Flag for the application layer to send the data. Make it true to send data.
//----------------------------------------------------
//Debug
//----------------------------------------------------

uint8_t fifoStatus = 0;
bool debugMsg = true;

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

    waitMicrosecond(5000);          // Wait for the nrf module to settle

    data = 0x0C;                    // Interrupts off,Enable CRC, 2 byte CRC
    writeNrfReg(W_REGISTER|CONFIG,data );

    data = 0x00;                    // Turn off retransmission   /*Check for using retransmission later*/
    writeNrfReg(W_REGISTER|SETUP_RETR,data );

    data = 0x03;                    // RF data rate to 1Mbps / 1 Mhz and power is -12 dbm with 7.5 mA consumption (medium power consumption)
    writeNrfReg(W_REGISTER|RF_SETUP,data );

    data = 0x05;                                // enable dynamic payloads, enable Tx_payload, NO_ACK
    writeNrfReg(W_REGISTER|FEATURE,data );

    data = 0x3F;                                // Configure dynamic payload lengths for all data pipes
    writeNrfReg(W_REGISTER|DYNPD,data );

    data = 0x70;
    writeNrfReg(W_REGISTER|STATUS, data);

    data = 76;                    // Set RF channel to be 2400 + 1(data) mhz
    writeNrfReg(W_REGISTER|RF_CH,data );

    //data = 0x03;                    // Interrupts off,no CRC, PWR_UP, Def Rx mode
    //writeNrfReg(W_REGISTER|CONFIG,data );

    //waitMicrosecond(1500); // Wait 1.5 ms for PWRUP

    data = 0x00;                    // Turn off auto ACKs
    writeNrfReg(W_REGISTER|EN_AA,data );
/*
// Turn on default Rx pipes
    data = 0x03;
    writeNrfReg(W_REGISTER|EN_RXADDR,data );


    data = 0x03;                    // 5byte Rx/Tx address width
    writeNrfReg(W_REGISTER|SETUP_AW,data );



    uint32_t pipeaddr = 0xAFAFAFAFAF;                    // Address for rx pipe 1

//    enableNrfCs();
//    writeSpi1Data(W_REGISTER|RX_ADDR_P0);
//    readSpi1Data();
//    writeSpi1Data(pipeaddr);
//    readSpi1Data();
//    disableNrfCs();

    enableNrfCs();
    writeSpi1Data(W_REGISTER|RX_ADDR_P0 +1);
    readSpi1Data();
    writeSpi1Data(pipeaddr);
    readSpi1Data();
    disableNrfCs();
//
//    data = 0xAF;
//    writeNrfReg(W_REGISTER|RX_ADDR_P0 +2,data );
//    writeNrfReg(W_REGISTER|RX_ADDR_P0 +3,data );
//    writeNrfReg(W_REGISTER|RX_ADDR_P0 +4,data );
//    writeNrfReg(W_REGISTER|RX_ADDR_P0 +5,data );

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

//Device slots

void joinAccessSlot_DEV(){
    txTimeStatus = true;
    slotNo = 3;
    acessSlotStart_dev = true;

    //debug
    putsUart0("JOINstart-D\n");
}
void uplinkSlot_DEV(){
    txTimeStatus = true;
    slotNo = eepromGetDevInfo_DEV() + 4;
    acessSlotStart_dev = false;
    uplinkSlot_dev = true; // Turn off after transmission
    appSendFlag = true;
    //debug
    putsUart0("uplinkstart-D\n");
}

void syncRxDevSlot(){
// Check if Sync was received by the device and wait for respective slots
    if(nrfJoinEnabled){
        stopTimer_ms(joinAccessSlot_DEV);
        startOneshotTimer_ms(joinAccessSlot_DEV, ACCESS_SLOT);
    }
    else{
        uint32_t myUplinkSlot = getMySlot(eepromGetDevInfo_DEV()); // Get the time you have to wait for your slotNo
        myUplinkSlot += GUARD_TIMER;
        stopTimer_ms(uplinkSlot_DEV);
        startOneshotTimer_ms(uplinkSlot_DEV, myUplinkSlot);
    }
    /*reset all the checks/data */
    isSync = false;
    uplinkSlot_dev = false;     // turned on by timer in its slotNo
    acessSlotStart_dev = false; // Turned on by timer if join enabled in device
    txTimeStatus = false; // Turned on by one shot timers
}



//Bridge Slots

void syncSlot_BR()
{
    nrfSyncEnabled = true;
    slotNo = 0;
    setPinValue(SYNC_LED,1); // Toggle SYNC for each sync sent
    putsUart0("\nSYNC-B\n");
}
void downlinkSlot_BR(){
    setPinValue(SYNC_LED,0); // Toggle SYNC for each sync sent
    slotNo = 1;
    txTimeStatus = true;
    downlinkSlotStart_br =true;
    putsUart0("DL-B\n");
}
void fastackSlot_BR(){
    txTimeStatus = true;
    slotNo = 2;
    downlinkSlotStart_br = false;
    fastackSlotStart_br = true;
    putsUart0("FACK-B\n");
}
void joinAccessSlot_BR(){
    txTimeStatus = true;
    slotNo = 3;
    fastackSlotStart_br = false;
    acessSlotStart_br = true;
    putsUart0("JOIN-B\n");
}
void uplinkSlot_BR(){
    acessSlotStart_br = false;
    uplinkSlot_br = false; // since uplink is not being used by bridge
    putsUart0("UL-B\n");
}

void TimerHandler_BR()
{
    if(!timersStarted_br){ // Start timers if not started
        stopTimer_ms(downlinkSlot_BR);
        stopTimer_ms(fastackSlot_BR);
        stopTimer_ms(joinAccessSlot_BR);
        stopTimer_ms(uplinkSlot_BR);

        putsUart0("ts -BR\n");
        startOneshotTimer_ms(downlinkSlot_BR, DL_SLOT);
        startOneshotTimer_ms(fastackSlot_BR, FACK_SLOT);
        startOneshotTimer_ms(joinAccessSlot_BR, ACCESS_SLOT);
        startOneshotTimer_ms(uplinkSlot_BR, UL0_SLOT);

        timersStarted_br = true;
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
    }
}

void enableJoin_BR()
{
    if(!getPinValue(JOIN_BUTTON)){ // wait for button press
        nrfJoinEnabled_BR = true;
        setPinValue(JOIN_LED, 1);
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
    writeNrfReg(W_REGISTER|CONFIG,0x0E); // CRC enabled, PWRUP, Prim_rx =0
    enableNrfCs(); // Frame begins
    writeNrfData(W_TX_PAYLOAD_NO_ACK);

    //Write data byte by byte
    for(i = 0, j =0; i < size ;i++, j++) {

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
    //CE enable to Rx mode//
    setPinValue(CE_GPIO,1);

    writeNrfReg(W_REGISTER|CONFIG,0x0F); // CRC enabled, PWRUP, Prim_Rx =1
    waitMicrosecond(TX_RX_DELAY);
    readNrfReg(R_REGISTER|R_RX_PL_WID, &payloadlength); // get the payload length of Rx packet


    enableNrfCs();                      // Frame begins
    writeNrfData(R_RX_PAYLOAD);

    while(i<payloadlength){
        readNrfData(&Rxpacket[Rx_wrIndex]);
        Rx_wrIndex = (Rx_wrIndex +1)%DATA_MAX_SIZE;
        ++i;
    }
    disableNrfCs();                     // Frame ends

    enableNrfCs();                              // Flush Tx FIFO after sending a SYNC
    writeSpi1Data(W_REGISTER|FLUSH_RX);
    readSpi1Data();
    disableNrfCs();
//    //debug
//    char str[32];
////    putsUart0("getdatapacket ");
//    for(i =0; i<payloadlength; i++){
//       snprintf(str, sizeof(str), "%"PRIu8, Rxpacket[i]);
//       putsUart0(str);
//       if(i%8==0){
//           putsUart0(" ");
//       }
//   }
//    putsUart0("\n");
}

void parsenrf24l01DataPacket(){
    // Parse the 32 bytes from getRxpacket and check for Sync, Start code, Slot number, Length , Data


    uint8_t deviceNum = 0;
    uint32_t devBits = 0;

    if(strncmp((char*)&Rxpacket[Rx_wrIndex - payloadlength], (char*)syncMsg, sizeof(syncMsg) -2) == 0){ // check sync. Last byte for frame count is omitted
        isSync = true;
        Rx_index = 0;
        Rx_wrIndex = 0;
        //Rx_index = (Rx_index + payloadlength)%DATA_MAX_SIZE; // increment read index
    }
    else{

        if(packetLength > 32 ){ // Check for continuos transmission of data
               putsUart0("more than 32 bytes\n");
        }
        else if(strncmp((char*)&Rxpacket[Rx_index], (char*)startCode, sizeof(startCode)) == 0){ // check start code
            deviceNum = eepromGetDevInfo_DEV(); //Get device number stored in EEPROm. For already stored devices

            if(!nrfSyncEnabled && !nrfJoinEnabled ) { // check whether myDevice got an ACK
                devBits = devBits | (Rxpacket[Rx_index +8] << 24);
                devBits = devBits | (Rxpacket[Rx_index +7] << 16);
                devBits = devBits | (Rxpacket[Rx_index +6] << 8);
                devBits = devBits | (Rxpacket[Rx_index +5]);

                if((devBits & (1 << deviceNum)) == 0 ) {
                    return;
                }

                putsUart0("the packet is for me....\n");
                if(Rxpacket[Rx_index +2] == 2){ // FACK slotNo
                   msgAcked = true;

                }
            }

            if(Rxpacket[Rx_index +4] != 0x80) {
                packetLength = Rxpacket[Rx_index+ 4] << 8; // MSB
            }
            packetLength |= Rxpacket[Rx_index +3];  // LSB

            if(Rxpacket[Rx_index + 2] == 1){ // DL slotNo

                if(Rxpacket[Rx_index +4] == 0x80) {
                    isJoinResp_dev = true;
                } else {
                    isDevPacket = true;
                    putsUart0("DL data received\n");
                }
                /*Used in setting dev no in downlink slotNo */
            }
            else if((Rxpacket[Rx_index +2] == 3) && nrfJoinEnabled_BR ){ // Msg in ACCESS slotNo

                if(Rxpacket[Rx_index +4] == 0x80) {
                      isJoinPacket = true;
                }
           }
            else if(Rxpacket[Rx_index +2] > 3){ // Msg in ACCESS slotNo
                isBridgePacket = true; // Is this packet received by the Bridge
            }

        }
        else{ // Error handling in case of junk data
            Rx_index =0;
            Rx_wrIndex =0;
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
   // readNrfReg(R_REGISTER|FIFO_STATUS,&data);
    readNrfReg(R_REGISTER|STATUS,&data);
    if(((data & 0x0E) == 0x0E) || ((data & 0x0E) == 0x0C)){ // check for the data pipe in RX_P_NO in FIFO_STATUS
        return false;
    }
    else{
        readNrfReg(R_REGISTER|STATUS,&data); // Read status register to check RX_DR
        writeNrfReg(R_REGISTER|STATUS,(data & ~0x40)); // Set the RX_DR bit to clear the interrupt
        return true;
    }
}

void nrf24l0PowerUp()
{
    uint8_t data;

    data = 0;
    readNrfReg(R_REGISTER|CONFIG, &data);

    if(!(data & 0x02)) {
        writeNrfReg(W_REGISTER|CONFIG, 0x0E); // CRC enabled, PWRUP, PRIM_RX=0
        waitMicrosecond(5000); // Wait 5 ms for checking that data write is complete

    }

}

bool isNrf24l0DataAvailable(void) // Check that data is available in Rx FIFO
{
    uint8_t data = 0;

    nrf24l0PowerUp();

    //readNrfReg(R_REGISTER|CONFIG, &data);
    writeNrfReg(W_REGISTER|CONFIG, 0x0F); // CRC enabled, PWRup, PRIM_RX=1

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
    uint8_t buffer[40] = {0};
    char str[40];
    putsUart0("data received to app layer\n");
    /*data copy*/
    for(i = 0; i < size; i++) {
        buffer[i] = data[i];
        snprintf(str,sizeof(str),"%u ",buffer[i]);
        putsUart0(str);
    }
    putsUart0("\n");
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

        syncMsg[sizeof(syncMsg) -2] = (uint8_t)((syncFrameCount & 0xFF00)>>8); // Frame count
        syncMsg[sizeof(syncMsg) -1] = (uint8_t)(syncFrameCount & 0x00FF);
        putNrf24l0DataPacket(syncMsg, sizeof(syncMsg));                                                         /*send sync message*/
        nrfSyncEnabled = false;
        ++syncFrameCount;
        uint32_t mySlotTemp = getMySlot(readEeprom(NO_OF_DEV_IN_BRIDGE));
        mySlotTemp += GUARD_TIMER;
        stopTimer_ms(syncSlot_BR);
        startOneshotTimer_ms(syncSlot_BR, mySlotTemp);
        timersStarted_br = false;
        Rx_index = 0;
        Rx_wrIndex = 0;
       // setPinValue(SYNC_LED, 1);
    }
}

void nrf24l0TxJoinReq_DEV()
{
    uint8_t joinBuffer[15] = {0};
    uint8_t* ptr = joinBuffer;
    uint8_t slotNo = 3; // Accessslot
    uint16_t remlen = (1+sizeof(myMac)) | (0x8000) ;

    strncpy((char*)ptr,(char*)startCode,sizeof(startCode));
    ptr += sizeof(startCode);
    strncpy((char*)ptr , (char*)&slotNo, sizeof(slotNo));
    ptr += sizeof(slotNo);
    strncpy((char*)ptr , (char*)&remlen, sizeof(remlen));
    ptr += sizeof(remlen);
    strncpy((char*)ptr , (char*)myMac, sizeof(myMac));
    ptr += sizeof(myMac);
    *ptr += nrf24l0GetChecksum(joinBuffer,sizeof(joinBuffer));

    //send join request in access slotNo
    putNrf24l0DataPacket(joinBuffer, sizeof(joinBuffer));
    txTimeStatus = false;
//    nrfJoinEnabled = false;
    putsUart0("joinrequest-D\n");
    acessSlotStart_dev = false;
}

void nrf24l0TxJoinResp_BR()
{
    uint8_t joinBuffer[10] = {0};
    uint8_t* ptr = joinBuffer;
    uint8_t slotNo = 1; // DL slotNo
    uint16_t remlen = ((1+1) | 0x8000); //crc + dev no

    strncpy((char*)ptr,(char*)startCode,sizeof(startCode));
    ptr += sizeof(startCode);
    strncpy((char*)ptr , (char*)&slotNo, sizeof(slotNo));
    ptr += sizeof(slotNo);
    strncpy((char*)ptr , (char*)&remlen, sizeof(remlen));
    ptr += sizeof(remlen);
    strncpy((char*)ptr , (char*)&allocatedDevNum, sizeof(allocatedDevNum));
    ptr += sizeof(allocatedDevNum);
    *ptr += nrf24l0GetChecksum(joinBuffer,sizeof(joinBuffer));

    //send join response in access slotNo//
    putNrf24l0DataPacket(joinBuffer, sizeof(joinBuffer));
    txTimeStatus = false;
    nrfJoinEnabled_BR = false;
    sendJoinResponse_BR = false;
    setPinValue(JOIN_LED, 0);

}

//void nrf24l0TxDownlink_BR()

int nrf24l0TxMsg(uint8_t* data, uint16_t size, uint32_t devBitNum)
{
    uint8_t packet[DATA_MAX_SIZE] = {0};
    uint8_t *ptr = packet;
    uint16_t remlen = size+1 + sizeof(devBitNum); /*Increment lenght for checksum */

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
        //user data//
        //Use devno for sending DL packets from Bridge
        // Device sends data to Bridge whose dev bit is 0xFFFFFFFF
        strncpy((char*)ptr , (char*)&devBitNum, sizeof(devBitNum));
        ptr += sizeof(devBitNum);

        strncpy((char*)ptr, (char*)data, size);
        ptr += size;

        //calculate checksum//
        *ptr = nrf24l0GetChecksum(packet,size + META_DATA_SIZE);

        putNrf24l0DataPacket(packet, size + META_DATA_SIZE);

        //guard timer //
        waitMicrosecond(GUARD_TIMER);

        enableNrfCs();                              // Flush Tx FIFO after sending
        writeSpi1Data(W_REGISTER|FLUSH_TX);
        readSpi1Data();
        disableNrfCs();

    return 0;
}



void nrf24l0RxMsg(callback fn)
{
    // Devices use this function as used in template code to keep on polling the data to check for received messages
    //**SYNC
    // Check for Sync message and start timers for the devices transmission slotNo
    // If a new device is Joining a timer is started to wait for transmission in ACCESS slotNo
    // If the bridge is transmitting (assumption that Bridge device also sends the Sync packets), Wait for Downlink and FASTACK slots to transmit data
    uint16_t size = 0;
    uint8_t Rxchecksum = 0; // Received checksum
    uint8_t devMac[6] = {0};
    uint16_t i,j = 0;
    char str[30];

    if(isNrf24l0DataAvailable()) {

        getnrf24l01DataPacket();
        parsenrf24l01DataPacket(); // set flags if syn message or startcode or

        if(isSync) //if sync //
        {
            syncRxDevSlot();                   // Handle device sync slots

            msgAcked = false;
            setPinValue(SYNC_LED,1);          // Toggle SYNC for each sync received
            waitMicrosecond(1000);
            setPinValue(SYNC_LED,0);
            putsUart0("SYNC-D\n");
        }

        else if(nrfJoinEnabled && isJoinResp_dev) //join response for dev
        {
            putsUart0("joinresponse-D\n");
            snprintf(str, sizeof(str), "allocated dev no: %"PRIu8"\n",Rxpacket[Rx_index + 5]);
            putsUart0(str);

            eepromSetDevInfo_DEV(Rxpacket[Rx_index + 5]);
            setPinValue(JOIN_LED,0);
            nrfJoinEnabled = false; // This is true from the moment dev sends a join req and receives a
            Rx_index += payloadlength - 1;
            isJoinResp_dev = false;
        }

        else if(msgAcked) //check acks for my device //
        {
            // Received ack . Stop retransmission. Send this to Application layer to stop retransmissting messages
            appSendFlag = false;
            Rx_index += payloadlength - 1;
        }

        else if(isJoinPacket){ // Bridge checks for packets received in Access slotNo if join button pressed on Bridge
//debug
putsUart0("isJoinPacket-BR\n");
            if(nrfJoinEnabled_BR && acessSlotStart_br)
            {
                for(i = 0, j = Rx_index + 5; i< 6; i++, j++)
                {
                    devMac[i] = Rxpacket[j];
                }
                sendJoinResponse_BR = true;
                allocatedDevNum =  eepromSetGetDevInfo_BR(devMac); // Set mac address of bridge in eeprom
                Rx_index += payloadlength - 1;

                putsUart0("Join Packet Req- BR\n");

            }
        }

        else if((isBridgePacket) ||(isDevPacket))
        {                                               // is data for me? and have I received whole packet*/
                                                        //crc check*/
            Rxchecksum = Rxpacket[Rx_index + payloadlength -1];
            //Rxchecksum = ~Rxchecksum;
            if(Rxchecksum == nrf24l0GetChecksum(&Rxpacket[Rx_index],payloadlength-1))
            {
                //Devices and Bridge can pop the data out and utilize it as needed
                size = payloadlength - META_DATA_SIZE;
                fn(&Rxpacket[Rx_index + META_DATA_SIZE - 1], size);                     // downlink data. Callback function */
                Rx_index = 0;
                Rx_wrIndex = 0; // reset for next packet Rx
                packetLength = 0;
                Rx_index += payloadlength - 1;
            }
                                                        //reset the buffer. */ //Extra precaution
            for (i=0; i<sizeof(Rxpacket); i++){
                Rxpacket[i] =0;
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

    return UL0_SLOT + (((50) +(TX_RX_DELAY_SLOT*_32BYTE_PACKETS) + GUARD_TIMER)*devno) ;

    //Handle slots for all transmissions for Devices and bridges separately
}

void eepromSetDevInfo_DEV(uint8_t deviceNum)
{
    uint8_t *ptr;
    uint8_t i;

    writeEeprom(NO_OF_DEV_IN_BRIDGE,1);                         // Number of devices is 1
    writeEeprom(DEV1_NO_START,deviceNum);                       // Device number

    ptr = (uint8_t*)DEV1_MAC_START;

    /*mac*/
    for(i = 0; i < sizeof(myMac); i++,++ptr) {
        writeEeprom(ptr, myMac[i]);
    }

}

uint8_t eepromGetDevInfo_DEV()
{
    return readEeprom(DEV1_NO_START); // Only one device for Dev; Device number
}

uint8_t eepromSetGetDevInfo_BR(uint8_t *data)
{
    int i=0;
    int j = 0;
    uint8_t devNo = 0;
    uint8_t* ptr = (uint8_t*)NO_OF_DEV_IN_BRIDGE;
    char str[30];
    uint8_t mac[6] = {0};

    uint8_t noOfDev = (readEeprom(NO_OF_DEV_IN_BRIDGE) == 0xFF) ? 0 : readEeprom(NO_OF_DEV_IN_BRIDGE);

    for(i = 0; i < noOfDev; i++) {
        ptr = (uint8_t*)DEV1_MAC_START;

        for(j = 0; j < 6; j++,ptr++){
            mac[j] = readEeprom(ptr);
        }

        if(strncmp((char*)mac,(char*)data,sizeof(myMac)) == 0) {
            ptr -= (sizeof(myMac) + 1);
            devNo = readEeprom(ptr);
            putsUart0("Existing Dev No: -BR:  ");
            snprintf(str, sizeof(str),"%u", devNo);
            putsUart0(str);
            putsUart0("\n");
            return devNo;
        }
        ptr += 1 + sizeof(myMac);
    }

    if(noOfDev == 0) {
        ptr = (uint8_t*)DEV1_MAC_START;
    }

    --ptr;
    ++noOfDev;
    writeEeprom(ptr, noOfDev);
    devNo = readEeprom(ptr);
    writeEeprom(NO_OF_DEV_IN_BRIDGE,devNo); /* store total no of devices */
    ++ptr;

    putsUart0("Dev No: -BR:  ");
    snprintf(str, sizeof(str),"%u", devNo);
    putsUart0(str);
    putsUart0("\n");

    /*mac*/
    for(i = 0; i < sizeof(myMac); i++,++ptr) {
        writeEeprom(ptr, data[i]);
    }

    return devNo;
}



void cmdHandler(){ // change this later if necessary
    bool end;
    char c;
    uint8_t i;
//    uint32_t* p32;

    #define MAX_CHARS 80
    char strInput[MAX_CHARS+1];
    char* token;
    uint8_t count = 0;

    if (kbhitUart0())
    {
        c = getcUart0();
        end = (c == 13) || (count == MAX_CHARS);

        if (!end)
        {
            if ((c == 8 || c == 127) && count > 0)
                count--;
            if (c >= ' ' && c < 127)
                strInput[count++] = c;
        }
        else
        {
            char str[32];

            strInput[count] = '\0';
            count = 0;
            token = strtok(strInput, " ");
            if (strcmp(token, "reboot") == 0)
            {
                writeNrfReg(W_REGISTER|CONFIG, 0x00); // Power down
                waitMicrosecond(5000);
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;

            }
            if(strcmp(token, "debug") == 0){
                debugMsg ^= true;
                putsUart0("Debug messages ON");
            }

            if (strcmp(token, "full") == 0)
            {
                putsUart0("fullpacket ");

                for(i =0; i<32; i++){
                    snprintf(str, sizeof(str), "%"PRIu8, Rxpacket[i]);
                    putsUart0(str);
                    if(i%8==0){
                        putsUart0(" ");
                    }
                }
                putsUart0("\n");
            }
        }
    }
}

/*
int main()
{
    initSystemClockTo40Mhz();

    initEeprom();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    nrf24l0Init();

    putsUart0("Device Powered up \n");
    // Flush Rx for device and bridge
    uint8_t data[] = {1,2,3,4,5,6}; // Random data, magic number change later
    enableSync_BR();

   // writeEeprom(NO_OF_DEV_IN_BRIDGE, 0);
    while(true)
    {
        cmdHandler();
        // Bridge Functions //
        nrf24l0TxSync();
        TimerHandler_BR();
        enableJoin_BR();

        //Common functions//
        nrf24l0RxMsg(dataReceived);

        //check for slotNo no//
        if(txTimeStatus) // change later
        {
            if(sendJoinResponse_BR && downlinkSlotStart_br){
                nrf24l0TxJoinResp_BR();
                }

            else if(appSendFlag){ // Used by application layer for controlling transmissions

                    if(downlinkSlotStart_br){
                        nrf24l0TxMsg(data,sizeof(data), 0); // Magic number devno change later to use Tx buffer pointer
                    }
                    if(fastackSlotStart_br){
                        nrf24l0TxMsg(data,sizeof(data), 0); // Magic number devno change later to use Tx buffer pointer
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
    initUart0();
    setUart0BaudRate(115200, 40e6);
    nrf24l0Init();

    putsUart0("Device Powered up \n");

    uint8_t data[] = {1,2,3,4,5,6,7}; // Random data, magic number change later

    //writeEeprom(NO_OF_DEV_IN_BRIDGE,0);                         // Number of devices is 1
    //writeEeprom(DEV1_NO_START,0);

    while(true)
        {
        cmdHandler();

    //Device functions
        enableJoin_DEV();
    //Common functions
        readNrfReg(R_REGISTER|FIFO_STATUS,&fifoStatus);
        nrf24l0RxMsg(dataReceived);

        //check for slotNo no
        if(txTimeStatus) // change later
        {
            if(nrfJoinEnabled && acessSlotStart_dev){
                nrf24l0TxJoinReq_DEV();
                setPinValue(JOIN_LED,1);
                }
            else if(appSendFlag && uplinkSlot_dev){ // Appsend flag is controlled by application layer.
                    nrf24l0TxMsg(data,sizeof(data), 0xFFFFFFFF); // Magic number devno change later to use TxFIFO buffer pointer
                    appSendFlag = false;
                    putsUart0("Uplink packet Sent\n");

            }
        }
    }
}




