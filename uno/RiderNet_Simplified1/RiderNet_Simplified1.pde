// This is for the first node on the xnp mesh network, works w/uno style arduinos
// Based off of XNP: https://github.com/SojournStudios/XBee_Network_Protocol
// robotgrrl was here, sorry i carved up your code kris! ;p

#include <NewSoftSerial.h>

NewSoftSerial mySerial(2,3);

int IDENTITY = 1;

#define MAX_BUFFER_LENGTH 192
int MAX_BUFFER_LENGTHS = 192;
int MAX_PACKET_LENGTHS = 98;
#define MAX_PACKET_LENGTH 98
#define PAYLOAD_LENGTH 32
int PAYLOAD_LENGTHS = 32;
#define HOST_LENGTH 16
int HOST_LENGTHS = 16;
#define SERIAL_LENGTH 12
int SERIAL_LENGTHS = 12;

uint32_t INTERVAL_USER_DATA = 50;
int ENABLE_PACKET_ENGINE = 1;
int MY_PACKET_DELINEATOR = 38;

uint32_t checkTime, serialNumber, discardPackets, inPackets;

int packetFlag, packetType, forwardPacket, bufferlen, parsePosition;
int packetLength, packetPositionPointer, foundDelineator, processingPacket, packetlen;
int z, f, s, x, y, c, a, i, b;

char buffer1[MAX_BUFFER_LENGTH], buffer2[MAX_BUFFER_LENGTH], packetbuffer[MAX_PACKET_LENGTH], buffer3[MAX_BUFFER_LENGTH];
char MY_ROUTER_ID[HOST_LENGTH], CONTROL_ROUTER[HOST_LENGTH], serialnum[HOST_LENGTH], psource[HOST_LENGTH], destination[HOST_LENGTH], pdest[HOST_LENGTH], source[HOST_LENGTH];
char payload[PAYLOAD_LENGTH], ppayload[PAYLOAD_LENGTH];

long MasterCount, serialNumberOffset;


void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  XNPsetHostsFile();  
}


void loop() {

  if(ENABLE_PACKET_ENGINE == 1) {

    XNPreadSerialBus();

    //if(IDENTITY > 1) {
      MasterCount++;
      if(MasterCount >= INTERVAL_USER_DATA) {
        XNPmySendingFunction();
        MasterCount = 0;
      }
    //}
    
  }
  
  delay(5);
  
}



// -------- setup

void XNPsetHostsFile() {
  sprintf(CONTROL_ROUTER, "Core");
  if (IDENTITY == 1) {
    sprintf(MY_ROUTER_ID, "Core");
  } else if(IDENTITY == 2) {
    sprintf(MY_ROUTER_ID, "Uno");
  }
}



// -------- rx

void XNPreadSerialBus() {
  
  s = 0;
  foundDelineator = 0;
  processingPacket = 0;
  packetPositionPointer = 0;
  sprintf(buffer3, "");

    if(mySerial.available())
  {
      while(mySerial.available())
    {
        byte inByte = mySerial.read();
      char inChar = char(inByte);
      int inInt = int(inChar);
      Serial.print(inChar); delay(5);
      sprintf(buffer2, "%c", (char)inByte);
      
      if(processingPacket == 1) {
        s++;
      }

      if((inInt < 32) || (inInt > 126)) {
        // Bad character, ignore it.
      } else if((processingPacket == 1) && (s >= MAX_PACKET_LENGTH)) {
        // Oops, packet is much larger than it should be - must be a mutant!
        
        Serial.println("drpd size is >>");
        
        processingPacket = 0;
        foundDelineator = 0;
        packetPositionPointer = 0;
        s = 0;
      } else if(inInt == MY_PACKET_DELINEATOR) {
        
        if(packetPositionPointer == 0) {
          delay(1);
          packetPositionPointer = 1;
          foundDelineator = 1;
        } else if(packetPositionPointer == 1) {
          delay(1);
          packetPositionPointer = 2;
          foundDelineator = 2;
        } else if(packetPositionPointer == 2) {
          delay(1);
          packetPositionPointer = 0;
          foundDelineator = 3;
        }

        if((foundDelineator == 3) && (processingPacket == 0)) {  
          delay(1);
          
          processingPacket = 1;
          foundDelineator = 0;
          packetPositionPointer == 0;
          s = 0;
          sprintf(packetbuffer, "");
        } else if((foundDelineator == 2) && (processingPacket == 1)) {
          delay(1);
          
          bufferlen = strlen(packetbuffer);
          if((bufferlen < 10) || (bufferlen >= MAX_PACKET_LENGTHS)) {
            
            Serial.println("pckt hdr/bdy sze outofbounds");

          } else {
          // Complete packet, send it on for processing in XNPprocessIncomingPacket()
            
            Serial.println("rd pckt"); // packetbuffer
            
            // Now send what Looks like a good packet to the XNP packet procssing engine for further analysis and reaction.
            XNPprocessIncomingPacket();
          }
          processingPacket = 0;
          packetPositionPointer = 0;
          foundDelineator = 0;
          s = 0;
        }
      } else if(processingPacket == 1) {
        // If we had found a packet delineator in the packet stream, but not a 2nd or 3rd after it, it means the 
        //  end of a character data field. We want to add this into the packet buffer for the parsing engine to recognize.
        if(packetPositionPointer > 0) {
          foundDelineator = 0;
          packetPositionPointer = 0;
          sprintf(buffer2, "%c", MY_PACKET_DELINEATOR);
          strcat(packetbuffer, buffer2);
        }
        sprintf(buffer2, "%c", (char)inByte);
        strcat(packetbuffer, buffer2);
      }
    }
  }
}

int XNPparseIncomingPacket() {
  
  int inInt;
  char inChar;
  byte inByte;

  // First pull-out the serial number
  sprintf(serialnum, "");
  parsePosition = 0;
  c = SERIAL_LENGTHS;
  for(a = parsePosition; a < c; a++) {
    parsePosition++;
    inByte = packetbuffer[a];
    inChar = char(inByte);
    inInt = int(inChar);
    sprintf(buffer2, "%c", packetbuffer[a]);
    if(inInt == MY_PACKET_DELINEATOR) {
      break;
    } else {
      strcat(serialnum, buffer2);
    }
  }
  // Convert the (char[16]) "serialnum" into an (unsigned long) "serialNumber" variable.
  serialNumber = atol(serialnum);
  if(serialNumber < 1) {
    Serial.println("serial num is bad, dropping");
    return(1);
  }

  // Next pull-out the source
  sprintf(source, "");
  c = parsePosition + HOST_LENGTHS;
  for(a = parsePosition; a < c; a++) {
    parsePosition++;
    inByte = packetbuffer[a];
    inChar = char(inByte);
    inInt = int(inChar);
    sprintf(buffer2, "%c", packetbuffer[a]);
    if(inInt == MY_PACKET_DELINEATOR) {
      break;
    } else {
      strcat(source, buffer2);
    }
  }
  bufferlen = strlen(source);
  if(bufferlen < 1) {
    Serial.println("source field is too small, dropping");
    return(1);
  }

  // If this packet is our own packet, stop reading right here - we don't need to process our own packets!
  if(!strcmp (source, MY_ROUTER_ID)) return(2);

  // Next pull-out the destination
  sprintf(destination, "");
  c = parsePosition + HOST_LENGTHS;
  for(a = parsePosition; a < c; a++) {
    parsePosition++;
    inByte = packetbuffer[a];
    inChar = char(inByte);
    inInt = int(inChar);
    sprintf(buffer2, "%c", packetbuffer[a]);
    if(inInt == MY_PACKET_DELINEATOR) {
      break;
    } else {
      strcat(destination, buffer2);
    }
  }
  bufferlen = strlen(destination);
  if(bufferlen < 1) {
    Serial.println("dst too small, droping");
    return(1);
  }

    // not entirely sure why this is here, works fine commented out
    /*
    if((!strcmp (destination, MY_ROUTER_ID)) || (!strcmp (destination, "ALLXNP"))) {
      // This packet is meant for us!
    } else {
      return(2);
    }
    */

  // Now get the packet type out.   
  packetType = 0;
  c = parsePosition + 3;
  i = 0;
  for(a = parsePosition; a < c; a++) {
    parsePosition++;
    inByte = packetbuffer[a];
    inChar = char(inByte);
    inInt = int(inChar);
    sprintf(buffer2, "%c", packetbuffer[a]);
    if(i == 0) {
      x = atoi(buffer2);
    } else if(i == 1) {
      y = atoi(buffer2);
    } else if(i == 2) {
      z = atoi(buffer2);
    }
    i++;
  }
  if(i == 1) {
    packetType = x;
  } else if(i == 2) {
    packetType = (x * 10) + y;
  } else if(i == 3) {
    packetType = (x * 100) + (y * 10) + z;
  }
  if((packetType < 1) || (packetType > 999)) {
    Serial.println("packet type bad, dropping");
    return(1);
  }

  // Next grab the packet flag.
  parsePosition++;
  packetFlag = 0;
  c = parsePosition + 3;
  i = 0;
  for(a = parsePosition; a < c; a++) {
    parsePosition++;
    inByte = packetbuffer[a];
    inChar = char(inByte);
    inInt = int(inChar);
    sprintf(buffer2, "%c", packetbuffer[a]);
    if(inInt == MY_PACKET_DELINEATOR) {
      break;
    } else {
      if(i == 0) {
        x = atoi(buffer2);
      } else if(i == 1) {
        y = atoi(buffer2);
      } else if(i == 2) {
        z = atoi(buffer2);
      }
      i++;
    }
  }
  if(i == 1) {
    packetFlag = x;
  } else if(i == 2) {
    packetFlag = (x * 10) + y;
  } else if(i == 3) {
    packetFlag = (x * 100) + (y * 10) + z;
  }
  if((packetFlag < 0) || (packetFlag > 999)) {
    Serial.println("packet flag bad, dropping");
    return(1);
  }

  // Last in the header; grab the packet length.
  parsePosition++;
  packetlen = 0;
  c = parsePosition + 3;
  i = 0;
  for(a = parsePosition; a < c; a++) {
    parsePosition++;
    inByte = packetbuffer[a];
    inChar = char(inByte);
    inInt = int(inChar);
    sprintf(buffer2, "%c", packetbuffer[a]);
    if(inInt == MY_PACKET_DELINEATOR) {
      break;
    } else {
      if(i == 0) {
        x = atoi(buffer2);
      } else if(i == 1) {
        y = atoi(buffer2);
      } else if(i == 2) {
        z = atoi(buffer2);
      }
      i++;
    }
  }
  if(i == 1) {
    packetlen = x;
  } else if(i == 2) {
    packetlen = (x * 10) + y;
  } else if(i == 3) {
    packetlen = (x * 100) + (y * 10) + z;
  }
  bufferlen = strlen(packetbuffer);
  if((bufferlen < 1) || (bufferlen > MAX_PACKET_LENGTHS)) {
    Serial.println("packet length out of bounds, dropping");
    return(1);
  }

  // Finally grab the packet payload.
  parsePosition++;
  c = bufferlen; // end of the packet!
  sprintf(payload, "");
  for(a = parsePosition; a < c; a++) {
    parsePosition++;
    inByte = packetbuffer[a];
    inChar = char(inByte);
    inInt = int(inChar);
    if(inInt == MY_PACKET_DELINEATOR) {
      break;
    } else {
      sprintf(buffer1, "%c", packetbuffer[a]);
      strcat(payload, buffer1);
    }
  }
  bufferlen = strlen(payload);
  if((bufferlen < 1) || (bufferlen > PAYLOAD_LENGTH)) {
    Serial.println("payload out of bounds, dropping");
    return(1);
  }

  // Calculate packet length and compare to received result.
  bufferlen = strlen(packetbuffer);
  if(bufferlen != packetlen) {
    Serial.println("packet len does not match header, dropping");
    return(1);
  }

  // Cool! We have a good packet, nicely chopped-up and ready for sale at the butchers!
  // ^ that's a freaky comment haha
  
  // let's just print this out nice and easy so that we don't have to use any more buffers
  if(true) {
  Serial.print("source: ");
  Serial.print(source);
  Serial.print(" dest: ");
  Serial.print(destination);
  Serial.print(" payload: ");
  Serial.print(payload);
  Serial.print(" serial num: ");
  Serial.print(serialNumber);
  Serial.print(" packet type: ");
  Serial.print(packetType);
  Serial.print(" flag: ");
  Serial.print(packetFlag);
  Serial.print(" len: ");
  Serial.print(packetlen);
  Serial.print("\n\n");
  }
  
   // sprintf(buffer1, "Parsed Packet: [%s], [%s], [%s], [%lu], [%d], [%d], len: %d", 
   //   source, destination, payload, serialNumber, packetType, packetFlag, packetlen);
   // XNPlogger(buffer1, 0);
  return(0);
}

void XNPprocessIncomingPacket() {
  
  a = XNPparseIncomingPacket();

  // First task is to parse the incoming packet, which will load the key data into buffer variables:
  // "source" transmitter, "destination" of packet, "serialNumber" of packet and the "payload"
  if(a == 2) {
    discardPackets++;
    // Packet we want to ignore... So, like, Ignore it and get back to work!
    return;
  } else if(a == 1) {
    // Bad packet! Chaulk up another victim and move along..

    return;
  } else {
    // Ahhh a Good packet! Process it with ... joy?
    inPackets++;

    // Is it a duplicate packet that we've seen before? The Input Buffer will tell us by comparing serial numbers.
    b = 0;
    if(!strcmp (source, MY_ROUTER_ID)) {   // Ack! This is a mirror-reflection of a packet we sent out - ignore it. (O_o)
      Serial.println("mirrored, ignoring");
      return;
    } else {
      // Is this packet meant for us?
      
      
      // same as in xnpparseincomingpacket- not too sure what to check for here
      
      /*
      if((!strcmp (destination, MY_ROUTER_ID)) || (!strcmp (destination, "ALLXNP"))) {

        Serial.println("Packet type: " + packetType);
        
        
        if(packetType == 2) {        // Sensor/Data Packet (DO NOT TOUCH)
          XNPprocessDataPacket();
        } else if(packetType == 3) { // Generic Receive function! Most common useage
          XNPmyReceiveFunction();
        } else if(packetType == 4) { // Cascade Reboot command (all routers should cycle power)
          XNPprocessNetworkPSTbounce();
        } else if(packetType == 5) { // Self Reboot command (reboot my local router/device only).
          XNPprocessSelfPSTbounce();
          sprintf(buffer1, "Received unknown packetType (Serial: %lu, Source: %s), Packet Type: %d", serialNumber, source, packetType);
          XNPlogger(buffer1, 0);
        }
        
        
        return;
      } else {
        // This packet is not meant for us! Determine what to do about it.
        Serial.println("hmm");
      }
      */
      
      
      
    }
  }
}


//------------ tx

void XNPmySendingFunction() {
    
  Serial.println("sending func"); delay(50);

  if(IDENTITY == 1) {
    sprintf(destination, "Robot-2");
  } else {
    sprintf(destination, "Robot-1");
  }
 
  sprintf(payload, "Hello World!");    

  XNPsendDataPacket(destination, payload, 3);
  
}


void XNPsendDataPacket(char *destin, char *stuff, int ptype) {
  
  Serial.print("send data pack");
  
  int q;
  forwardPacket = 0;

  XNPcreateSerialNumber();
  sprintf(psource, "%s", MY_ROUTER_ID);
  sprintf(pdest, "%s", destin);

  bufferlen = strlen(stuff);
  if(bufferlen > PAYLOAD_LENGTHS) {
    sprintf(buffer2, "");
    for(q = 0; q < PAYLOAD_LENGTHS; q++) {
      sprintf(buffer1, "%c", stuff[q]);
      strcat(buffer2, buffer1);
    }
    sprintf(ppayload, "%s", buffer2);
  } else if(bufferlen == 0) {
    sprintf(ppayload, "%s0", stuff);
  } else {
    sprintf(ppayload, "%s", stuff);
  }
  packetFlag = 0;
  packetType = ptype;
  XNPloadPacketBuffer();
  
  Serial.println("...et");
  
}

void XNPloadPacketBuffer() {
    // First handle the case where we don't think, we just scream (O_O).
    Serial.println("hihi");
    //mySerial.print('!!');
   
    if(XNPmakePacketSandwich() != 1) {
    //mySerial.print('!!');
    XNPtrasnmitPacket();
    //Serial.print('!!'); delay(5);
    } else {
      Serial.print("failed");
      return; 
    }
}

void XNPtrasnmitPacket() {
  sprintf(buffer3, "");
  bufferlen = strlen(packetbuffer);
  for(f = 0; f < bufferlen; f++) {
    sprintf(buffer2, "%c", packetbuffer[f]);

      byte outByte = (byte)buffer2[0];
      int outInt = (int)outByte;
      mySerial.print(buffer2);

      //Serial.print("Buffer2: "); delay(5);
      //Serial.print(buffer2); delay(5);

    strcat(buffer3, buffer2);
    }
}

int XNPmakePacketSandwich()
{
  sprintf(buffer2, "%c", MY_PACKET_DELINEATOR);
  sprintf(packetbuffer, "");
  // Add the packet serial number (with padding out to SERIAL_LENGTHS)
  sprintf(buffer1, "%lu", serialNumber);
  bufferlen = strlen(buffer1);
  if(bufferlen >= SERIAL_LENGTHS) {
    for(z = 0; z < SERIAL_LENGTHS; z++) {
      sprintf(buffer3, "%c", buffer1[z]);
      strcat(packetbuffer, buffer3);
    }
  } else {
    strcat(packetbuffer, buffer1);
  }
  strcat(packetbuffer, buffer2);

  // Add the packet source (with padding out to HOST_LENGTHS)
  bufferlen = strlen(psource);
  if(bufferlen >= HOST_LENGTHS) {
    for(z = 0; z < HOST_LENGTHS; z++) {
      sprintf(buffer3, "%c", psource[z]);
      strcat(packetbuffer, buffer3);
    }
  } else {
    strcat(packetbuffer, psource);
  }
  strcat(packetbuffer, buffer2);

  // Add the packet destination (with padding out to HOST_LENGTHS)
  bufferlen = strlen(pdest);
  if(bufferlen >= HOST_LENGTHS) {
    for(z = 0; z < HOST_LENGTHS; z++) {
      sprintf(buffer3, "%c", pdest[z]);
      strcat(packetbuffer, buffer3);
    }
  } else {
    strcat(packetbuffer, pdest);
  }
  strcat(packetbuffer, buffer2);

  // Add the packet type (with padding out to 3 max digits)
  if(packetType < 10) {
    sprintf(buffer3, "00%d", packetType);
  } else if(packetType < 100) {
    sprintf(buffer3, "0%d", packetType);
  } else {
    sprintf(buffer3, "%d", packetType);
  }
  strcat(packetbuffer, buffer3);
  strcat(packetbuffer, buffer2);

  // Add the packet flags (with padding out to 3 max digits)
  sprintf(buffer3, "%d", packetFlag);
  if(packetFlag < 10) {
    sprintf(buffer3, "00%d", packetFlag);
  } else if(packetFlag < 100) {
    sprintf(buffer3, "0%d", packetFlag);
  } else {
    sprintf(buffer3, "%d", packetFlag);
  }
  strcat(packetbuffer, buffer3);
  strcat(packetbuffer, buffer2);

  // Add the packet length (with padding out to 3 max digits)
  packetLength = (strlen(ppayload) + strlen(packetbuffer)) + 4;
  sprintf(buffer3, "%d", packetLength);
  if(packetLength < 10) {
    sprintf(buffer3, "00%d", packetLength);
  } else if(packetLength < 100) {
    sprintf(buffer3, "0%d", packetLength);
  } else {
    sprintf(buffer3, "%d", packetLength);
  }
  strcat(packetbuffer, buffer3);
  strcat(packetbuffer, buffer2);

  // Now add our payload to the packet!
  bufferlen = strlen(ppayload);
  if(bufferlen >= PAYLOAD_LENGTHS) {
    for(z = 0; z < PAYLOAD_LENGTHS; z++) {
      sprintf(buffer3, "%c", ppayload[z]);
      strcat(packetbuffer, buffer3);
    }
  } else {
    strcat(packetbuffer, ppayload);
  }

  // Now calculate the packet length and insert it into the header (goes at the end of the header).
  packetLength = strlen(packetbuffer);
  if(packetLength > MAX_PACKET_LENGTHS) {
    Serial.println("pckt sndwch too big");
    return(1);
  } else {
    // Add the length, pop-on 3 delineators to mark the beginning of the packet.
    sprintf(buffer3, "%c%c%c", MY_PACKET_DELINEATOR, MY_PACKET_DELINEATOR, MY_PACKET_DELINEATOR);
    sprintf(buffer1, "%s%s", buffer3, packetbuffer);
    sprintf(packetbuffer, "%s", buffer1);
    // Next and 2 delineators to mark the end and we're done!
    sprintf(buffer3, "%c%c.", MY_PACKET_DELINEATOR, MY_PACKET_DELINEATOR);
    strcat(packetbuffer, buffer3);
    return(0);
  }
}



// --------

void XNPcreateSerialNumber()
{
  checkTime = millis();
  checkTime += serialNumberOffset;
  sprintf(buffer3, "%lu", checkTime);
  x = SERIAL_LENGTHS - 1;
  bufferlen = strlen(buffer3);
  if(bufferlen > x) {
    sprintf(serialnum, "");
    for(y = 0; y < x; y++) {
      sprintf(buffer2, "%c", buffer3[y]);
      strcat(serialnum, buffer2);
    }
  } else {
    sprintf(serialnum, "%s", buffer3);
  }

  // Now convert the 16-character string into a long unsigned int in "serialNumber"
  serialNumber = atol(serialnum);    // Thank You Len17! :)  (http://forums.adafruit.com/viewtopic.php?f=25&t=25264) 
}

