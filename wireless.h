

#ifndef WIRELESS_H_
#define WIRELESS_H_

// Message types and params
#define NRF_SYNC        (0u)
#define NRF_DATA        (1)
#define NRF_JOIN        (2)

#define NO_MSGS         (10)
#define MSG_LEN         (25)
#define DEF_SYNC_TIME   (5)         // Default Sync message time

//DATA transmission
#define DATA_MAX_SIZE   (1500u)
#define META_DATA_SIZE  (2+1+2+1)   //startcode + slot + len + checksum

//EEPROM
#define NO_OF_DEV_IN_BRIDGE     (10u)   /*1 word */
#define DEV1_NO_START           (14u)
#define DEV1_MAC_START          (15u)
#define DEV2_NO_START           (19u)
#define DEV2_MAC_START          (20u)

//NRF Register addresses
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RX_ADDR_P0  0x0A            // rx pipe address. add offsets for following pipes
#define TX_ADDR     0x10
#define RX_PW_P0    0x11            // number of bytes allowed in rx pipe 0. Add offsets for following pipes
#define FIFO_STATUS 0x17
#define DYNPD       0x1C            // enable dynamic payload for diff tx pipes
#define FEATURE     0x1D

//SPI Commands
#define R_REGISTER          (0x00)
#define W_REGISTER          (0x20)
#define R_RX_PAYLOAD        (0x61)
#define W_TX_PAYLOAD        (0xA0)
#define W_TX_PAYLOAD_NO_ACK (0xB0)
#define FLUSH_TX            (0xE1)
#define FLUSH_RX            (0xE2)
#define R_RX_PL_WID         (0x60)


//Wireless PINS
#define CE_GPIO         PORTB,1
#define JOIN_BUTTON     PORTF,4
#define SYNC_LED        PORTF,1
#define JOIN_LED        PORTF,2


//Slot Management Macros
//#define GUARD_TIMER (60u)
//
//#define SYNC_SLOT   (0)
//#define DL_SLOT     (60u) + GUARD_TIMER
//#define FACK_SLOT   (21000u) + DL_SLOT+ GUARD_TIMER
//#define ACCESS_SLOT (100u) + FACK_SLOT + GUARD_TIMER
//#define UL0_SLOT    (110u) + ACCESS_SLOT + GUARD_TIMER
//#define UL1_SLOT    (21000u) + UL0_SLOT + GUARD_TIMER

#define GUARD_TIMER (1u) // minimum is 1 ms

#define SYNC_SLOT   (0)
#define DL_SLOT     (2) + GUARD_TIMER
#define FACK_SLOT   (22) + DL_SLOT+ GUARD_TIMER
#define ACCESS_SLOT (2) + FACK_SLOT + GUARD_TIMER
#define UL0_SLOT    (2) + ACCESS_SLOT + GUARD_TIMER
#define UL1_SLOT    (22) + UL0_SLOT + GUARD_TIMER


#define TX_RX_DELAY (130 + 70) //microseconds

//Extern variables

// Declare external variables for nrfSyncEnabled, nrfJoinEnabled and nrfJoinEnabled_BR
extern bool nrfSyncEnabled;
extern bool nrfJoinEnabled;
extern bool nrfJoinEnabled_BR;

// Declare external variables for syncMsg, startCode, myMac
extern uint8_t syncMsg[6];
extern uint8_t startCode[2];
extern uint8_t myMac[6];

// Declare external variables for slotNo, syncFrameCount, packetLength, allocatedDevNum
extern uint8_t slotNo;
extern uint16_t syncFrameCount;
extern uint16_t packetLength;
extern uint8_t allocatedDevNum;

// Declare external variables for Rxpacket, Rx_index
extern uint8_t Rxpacket[DATA_MAX_SIZE];
extern uint16_t Rx_index;

// Declare external variables for isSync, isMyPacket, msgAcked, isDevPacket, isBridgePacket, isJoinPacket
extern bool isSync;
extern bool isMyPacket;
extern bool msgAcked;
extern bool isDevPacket;
extern bool isBridgePacket;
extern bool isJoinPacket;

// Declare external variables for downlinkSlotStart_br, fastackSlotStart_br, acessSlotStart_br, uplinkSlot_br, sendJoinResponse_BR
extern bool downlinkSlotStart_br;
extern bool fastackSlotStart_br;
extern bool acessSlotStart_br;
extern bool uplinkSlot_br;
extern bool sendJoinResponse_BR;

// Declare external variables for uplinkSlot_dev, acessSlotStart_dev
extern bool uplinkSlot_dev;
extern bool acessSlotStart_dev;

// Declare external variable for txTimeStatus
extern bool txTimeStatus;



//Subroutines

typedef void (*callback)(uint8_t *data, uint16_t size);

//----------------------------------------------------
/* Wireless Global common subroutines */
//----------------------------------------------------

void nrf24l0Init();
int nrf24l0TxMsg(uint8_t* data, uint16_t size,uint8_t devno);
void nrf24l0RxMsg(callback fn);

//----------------------------------------------------
/* Wireless Global Device subroutines */
//----------------------------------------------------
void enableJoin_DEV(); // Enable for sending a Join req in Access slot on PB press
void eepromSetDevInfo_DEV(uint8_t deviceNum); // Set the Device number received as Join response in EEPROM
uint8_t eepromGetDevInfo_DEV(); // Get device number stored in EEPROM


//----------------------------------------------------
/* Wireless Global Bridge subroutines */
//----------------------------------------------------

void enableSync_BR(); // Send a Sync Message
uint8_t eepromSetGetDevInfo_BR(uint8_t *data); // Allocate a device num for new device OR Get device number if already available

//----------------------------------------------------
// Timer Internal functions
//----------------------------------------------------
//BR timer callback functions
void syncSlot_BR();
void downlinkSlot_BR();
void fastackSlot_BR();
void joinAccessSlot_BR();
void uplinkSlot_BR();

//Device timer callback functions
void joinAccessSlot_DEV();
void uplinkSlot_DEV();

//Timing handler functions
void TimerHandler_BR();
void syncRxDevSlot();

//----------------------------------------------------
/*Wireless Internal subroutines*/
//----------------------------------------------------

//Initialize
void nrf24l0ChipInit();

//SPI Bus
void enableNrfCs(void);
void disableNrfCs(void);
void writeNrfReg(uint8_t reg, uint8_t data);
void writeNrfData(uint8_t data);
void readNrfReg(uint8_t reg, uint8_t *data);
void readNrfData(uint8_t *data);
void nrf24l0PowerUp();

//Slot management
uint32_t getMySlot(uint8_t devno);

//----------------------------------------------------
// Tx/ Rx functions
//----------------------------------------------------
void putNrf24l0DataPacket(uint8_t *data, uint16_t size);
void getnrf24l01DataPacket();
void parsenrf24l01DataPacket();
bool nrf24l0TxStatus();
bool nrf24l0RxStatus();
bool isNrf24l0DataAvailable(void);

//Callback functions
callback dataReceived(uint8_t *data, uint16_t size);

//Checksum functions
uint8_t nrf24l0GetChecksum(uint8_t *ptr, uint16_t size);

//Tx Msg handlers for SYNC, JOINREQ, JOINRESP
void nrf24l0TxSync();
void nrf24l0TxJoinReq_DEV();
void nrf24l0TxJoinResp_BR();


#endif /* WIRELESS_H_ */
