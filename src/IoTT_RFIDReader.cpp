#include <IoTT_RFIDReader.h>
#include <IoTT_DigitraxBuffers.h>

void printHexData(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

IoTT_RFIDReader::IoTT_RFIDReader()
{
}

IoTT_RFIDReader::~IoTT_RFIDReader()
{
	freeObjects();
}

void IoTT_RFIDReader::setTxFunction(txFct newFct)
{
	sendLNMessage = newFct;
}

void IoTT_RFIDReader::loadRFIDReaderCfgJSON(JsonObject thisObj, bool initAll)
{
	pinSCK = thisObj["rfidSCK"];
    pinSDA_SS = thisObj["rfidSDA_SS"];
    pinMISO = thisObj["rfidMISO"];
    pinMOSI = thisObj["rfidMOSI"];
    pinRST = thisObj["rfidRST"];
	readerID = thisObj["readerID"];
	tagReqBroadcast = thisObj["tagReqBroadcast"];
	sendMultiSense = thisObj["sendMultiSense"];
	
	//add code to read tag map
    for (byte i = 0; i < 6; i++) 
	#ifdef UseMFRC522
		key.keyByte[i] = 0xFF;
	#endif
	#ifdef UsePN532
//		key[i] = 0xFF;
	#endif
	if (initAll)
	{
		initSPI();
		initReader();
	}
}

void IoTT_RFIDReader::loadRFIDTagMapJSON(DynamicJsonDocument doc)
{
	JsonArray tagMapArray = doc["TagMap"];
	tagMapListLen = tagMapArray.size();
	tagMapList = (tagDataType*) realloc (tagMapList, tagMapListLen * sizeof(tagDataType));
    for (uint16_t i = 0; i < tagMapListLen; i++) 
    {
		JsonObject thisTag = tagMapArray[i];
		uint32_t thisTagID = thisTag["TagID"];
		for (uint8_t j = 0; j < 8; j++) 
			tagMapList[i].tagData[j] = (thisTagID >> (8*j)) & 0xFF;
		tagMapList[i].dccAddr = thisTag["DCCAddr"];
	}
}

void IoTT_RFIDReader::initSPI()
{
	Serial.printf("Init SPI with SCK %i MISO %i MOSI %i SDA %i\n", pinSCK, pinMISO, pinMOSI, pinSDA_SS);
	SPI.begin(pinSCK, pinMISO, pinMOSI, pinSDA_SS); // Init SPI bus
}

void IoTT_RFIDReader::initReader()
{
	Serial.printf("Init RFID Reader with SDA %i RST %i\n", pinSDA_SS, pinRST);
	#ifdef UseMFRC522
		myMFRC = new MFRC522(pinSDA_SS, pinRST);
		myMFRC->PCD_Init(); // Init MFRC522 
    #endif
	#ifdef UsePN532
//		myMFRC = new Adafruit_PN532(pinSCK, pinMISO, pinMOSI, pinSDA_SS);
//		myMFRC->begin(); // Init PN532 
    #endif
}

void IoTT_RFIDReader::freeObjects()
{
	if (tagMapListLen > 0)
	{
		tagMapListLen = 0;
		free(tagMapList);
	}
}

bool IoTT_RFIDReader::performSelfTest()
{
#ifdef UseMFRC522	
	if (myMFRC)
	{
//		bool hlpBool = myMFRC->PCD_PerformSelfTest();
//		if (hlpBool)
			myMFRC->PCD_DumpVersionToSerial();
		return true;//hlpBool;
	}
	else
		return false;
#endif
#ifdef UsePN532	
//  uint32_t versiondata = myMFRC->getFirmwareVersion();
//  if (versiondata) 
//  {
//		Serial.printf("RFID PN532 V. %X Sw V. %i.%i\n", ((versiondata>>24) & 0xFF), ((versiondata>>16) & 0xFF), ((versiondata>>8) & 0xFF));
//		return true;
//  }
//	else
//		return false;
#endif
}

#ifdef UseMFRC522	
void IoTT_RFIDReader::scanRFID()
{
	if (myMFRC)
	{
		if (myMFRC->PICC_IsNewCardPresent()) 
		{
//			Serial.println("Card detected");
			if (myMFRC->PICC_ReadCardSerial()) 
			{
//				Serial.println("Card Read");
//				MFRC522::PICC_Type piccType = myMFRC->PICC_GetType(myMFRC->uid.sak);
//        		Serial.println(myMFRC->PICC_GetTypeName(piccType));
				if (myMFRC->uid.uidByte[0] != lastTag.tagData[0] || myMFRC->uid.uidByte[1] != lastTag.tagData[1] || myMFRC->uid.uidByte[2] != lastTag.tagData[2] || myMFRC->uid.uidByte[3] != lastTag.tagData[3] || (millis() > lastTag.tagTime + 2000)) 
				{
					for (byte i = 0; i < 4; i++)
						lastTag.tagData[i] = myMFRC->uid.uidByte[i];
					transmitReadEvent(lastTag);
//					printHexData(myMFRC->uid.uidByte, myMFRC->uid.size);
//					Serial.println(" UID");
					if (sendMultiSense)
						for (uint16_t i = 0; i < tagMapListLen; i++)
						{
//							printHexData(tagMapList[i].tagData, 4);
//							Serial.println(" Tag");
							if ((tagMapList[i].tagData[0] == lastTag.tagData[0]) && (tagMapList[i].tagData[1] == lastTag.tagData[1]) && (tagMapList[i].tagData[2] == lastTag.tagData[2]) && (tagMapList[i].tagData[3] == lastTag.tagData[3]))// && (millis() > lastTag.tagTime + 2000)) 
							{
								lastTag.dccAddr = tagMapList[i].dccAddr;
								sendMultiSenseMsg(tagMapList[i]);
								tagMapList[i].tagTime = millis();
							}
						}
					lastTag.tagTime = millis();
				}
				myMFRC->PICC_HaltA();
				myMFRC->PCD_StopCrypto1();
			}
		}
	}
}
#endif

/*
#ifdef UsePN532
void IoTT_RFIDReader::scanRFID()
{
	uint8_t success;
	uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
	uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
	if (myMFRC)
	{
    
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
		success = myMFRC->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
		if ((success) && (uidLength == 4))
		{
				if (uid[0] != lastTag.tagData[0] || uid[1] != lastTag.tagData[1] || uid[2] != lastTag.tagData[2] || uid[3] != lastTag.tagData[3] || (millis() > lastTag.tagTime + 2000)) 
				{
					for (byte i = 0; i < 4; i++)
						lastTag.tagData[i] = uid[i];
					transmitReadEvent(lastTag);
//					printHexData(myMFRC->uid.uidByte, myMFRC->uid.size);
//					Serial.println(" UID");
					if (sendMultiSense)
						for (uint16_t i = 0; i < tagMapListLen; i++)
						{
//							printHexData(tagMapList[i].tagData, 4);
//							Serial.println(" Tag");
							if ((tagMapList[i].tagData[0] == lastTag.tagData[0]) && (tagMapList[i].tagData[1] == lastTag.tagData[1]) && (tagMapList[i].tagData[2] == lastTag.tagData[2]) && (tagMapList[i].tagData[3] == lastTag.tagData[3]))// && (millis() > lastTag.tagTime + 2000)) 
							{
								lastTag.dccAddr = tagMapList[i].dccAddr;
								sendMultiSenseMsg(tagMapList[i]);
								tagMapList[i].tagTime = millis();
							}
						}
					lastTag.tagTime = millis();
				}
			
		}
	}
}
#endif
*/

void IoTT_RFIDReader::processLocoNetMsg(lnReceiveBuffer * newData)
{
	switch (cmdCCC(newData->lnData[15]))
	{
		case CMD_RDR_ERASE: //IN erase tag map if for this reader_id
			if ((newData->lnData[16] == readerID) || (newData->lnData[16] == 0))
				eraseTagTable();
			break;
		case CMD_RDR_PING: //IN reply with CMD_RDR_ACK if for this reader_id or broadcast
			if ((newData->lnData[16] == readerID) || (newData->lnData[16] == 0))
				sendPing();
			break; 
		case CMD_RDR_ACK:  //OUT this is someone elses ping answer. Ignor unless you want to keep track of all readers in the network
			break;
		case CMD_TAG_READ: //OUT this is someone elses read report. Ignor unless you want to keep track of all data
			break;
		case CMD_TAG_MAP: //IN add a new entry to the tag map (mapping RFID data to 15 bit DCC addresses
			addTagToTable(newData);
			break;
		case CMD_TAG_REQ: //OUT this is someone elses request to learn a tag map. Ignore unless tagReqBroadcast == true and tag ID is in mapping table
			if ((newData->lnData[16] != readerID) && tagReqBroadcast)
				sendTagData(newData);
			break;
		default: break;
	}
}

void IoTT_RFIDReader::sendMultiSenseMsg(tagDataType thisTag)
{
	lnTransmitMsg newMsg;
	newMsg.lnMsgSize = 6;
	newMsg.lnData[0] = 0xD0;
	newMsg.lnData[1] = 0x67;
	newMsg.lnData[2] = readerID;
	newMsg.lnData[3] = (thisTag.dccAddr & 0x3F80) >> 7;
	newMsg.lnData[4] = (thisTag.dccAddr & 0x7F);
	newMsg.lnData[5] = 0xD0;


	for (uint8_t i = 1; i < newMsg.lnMsgSize-1; i++)
	  newMsg.lnData[newMsg.lnMsgSize-1] ^= newMsg.lnData[i];
	newMsg.lnData[newMsg.lnMsgSize-1] ^= 0xFF;
	if (sendLNMessage != NULL)
	{
		sendLNMessage(newMsg);
	} 
}

void IoTT_RFIDReader::sendRFIDMsg(uint8_t thisReader, cmdCCC thisCmd, uint8_t thisRfidData[8], uint16_t thisDccAddr)
{
	lnTransmitMsg newMsg;
	newMsg.lnMsgSize = 20;
	newMsg.lnData[0] = 0xE5;
	newMsg.lnData[1] = 0x14;
	newMsg.lnData[2] = 0x06;
	newMsg.lnData[3] = 0x20;
	newMsg.lnData[4] = 0x00;
	for (uint8_t i = 0; i < 4; i++)
	{
		newMsg.lnData[i+5] = (thisRfidData[7-i] & 0x7F);
		newMsg.lnData[4] |= (thisRfidData[7-i] & 0x80) >> (7-i);
	}
	newMsg.lnData[9] = 0x00;
	for (uint8_t i = 0; i < 4; i++)
	{
		newMsg.lnData[i+10] = (thisRfidData[3-i] & 0x7F);
		newMsg.lnData[9] |= (thisRfidData[3-i] & 0x80) >> (7-i);
	}
	newMsg.lnData[14] = 0x00; //always 0 as all following bytes are 7bit only
	newMsg.lnData[15] = thisCmd;
	newMsg.lnData[16] = readerID;
	newMsg.lnData[17] = (thisDccAddr & 0x3F80) >> 7;
	newMsg.lnData[18] = (thisDccAddr & 0x007F);
	newMsg.lnData[19] = newMsg.lnData[0];
	for (uint8_t i = 1; i < newMsg.lnMsgSize-1; i++)
	  newMsg.lnData[newMsg.lnMsgSize-1] ^= newMsg.lnData[i];
	newMsg.lnData[newMsg.lnMsgSize-1] ^= 0xFF;
	if (sendLNMessage != NULL)
	{
		sendLNMessage(newMsg);
	} 
}

void IoTT_RFIDReader::sendPing()
{
	uint8_t rfidData[8] = {0,0,0,0,0,0,0,0};
	sendRFIDMsg(readerID, CMD_RDR_ACK, rfidData, 0);
}

void IoTT_RFIDReader::eraseTagTable()
{
}

bool IoTT_RFIDReader::addTagToTable(lnReceiveBuffer * newData)
{
}

void IoTT_RFIDReader::sendTagData(lnReceiveBuffer * newData)
{
}

void IoTT_RFIDReader::transmitReadEvent(tagDataType thisTag)
{
	sendRFIDMsg(readerID, CMD_TAG_READ, thisTag.tagData, thisTag.dccAddr);
}

/*----------------------------------------------------------------------------------------------------------------------*/

