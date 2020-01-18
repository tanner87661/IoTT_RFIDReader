#ifndef IoTT_RFIDReader_h
#define IoTT_RFIDReader_h


#define UseMFRC522
//#define UsePN532

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#ifdef UseMFRC522
	#include <MFRC522.h>
#endif
#ifdef UsePN532
//	#include <Adafruit_PN532.h>
#endif

#include <IoTT_DigitraxBuffers.h>
#include <IoTTCommDef.h>


enum cmdCCC : uint8_t {CMD_RDR_ERASE=1, CMD_RDR_PING=2, CMD_RDR_ACK=3, CMD_TAG_READ=4, CMD_TAG_MAP=5, CMD_TAG_REQ=6};

typedef struct {
	uint8_t tagData[8];
	uint16_t dccAddr;
	uint32_t tagTime;
} tagDataType;

class IoTT_RFIDReader
{
	public:
		IoTT_RFIDReader();
		~IoTT_RFIDReader();
		void loadRFIDReaderCfgJSON(JsonObject thisObj, bool initAll = false);
		void loadRFIDTagMapJSON(DynamicJsonDocument doc);
		void initSPI();
		void initReader();
		void scanRFID();
		bool performSelfTest();
		void processLocoNetMsg(lnReceiveBuffer * newData);
		void setTxFunction(txFct newFct);
	private:
		void freeObjects();
		void eraseTagTable();
		void sendPing();
		bool addTagToTable(lnReceiveBuffer * newData);
		void sendTagData(lnReceiveBuffer * newData);
		void transmitReadEvent(tagDataType thisTag);
		void sendMultiSenseMsg(tagDataType thisTag);
		void sendRFIDMsg(uint8_t thisReader, cmdCCC thisCmd, uint8_t thisRfidData[8], uint16_t thisDccAddr = 0);
	
	public:
	private:
		tagDataType lastTag;
		tagDataType * tagMapList = NULL;
		uint16_t tagMapListLen = 0;
		txFct sendLNMessage = NULL;
		#ifdef UseMFRC522
			MFRC522 * myMFRC = NULL;
			MFRC522::MIFARE_Key key; 
		#endif
		#ifdef UsePN532
//			Adafruit_PN532 * myMFRC = NULL;
//			uint8_t key[6];
		#endif
        uint8_t pinSCK = 0;
        uint8_t pinSDA_SS = 0;
        uint8_t pinMISO = 0;
        uint8_t pinMOSI = 0;
        uint8_t pinRST = 0;
		uint8_t readerID = 0;
		bool tagReqBroadcast = false;
		bool sendMultiSense = false;


};


//extern void sendSwitchCommand(uint16_t swiNr, uint8_t swiTargetPos, uint8_t coilStatus) __attribute__ ((weak)); //switch
//extern void sendSignalCommand(uint16_t signalNr, uint8_t signalAspect) __attribute__ ((weak)); //signal

#endif
