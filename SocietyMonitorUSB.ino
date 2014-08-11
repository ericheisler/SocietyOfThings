/*
This is the monitor.
It is connected to a compy via usb.

It begins by requesting ack from each node. Then sends nodeCount and begin signal.
All parameters are periodically polled from each node and sent to the compy.

Also, this can act as a regular node.

IR messages are formatted thus.
destination ID(char), source ID(char), message type(char), message(depends on type)

The monitor will have ID=255. Nodes will have IDs already programmed.
Message types 0 to 4 are only sent by monitor.
Message types include:
0 = request ack, followed by 1 byte to check signal
1 = set node count, followed by 1 byte (destination ID doesn't matter)
2 = begin conversing, followed by zero bytes (destination ID doesn't matter)
3 = reset all parameters, followed by zero bytes (destination ID doesn't matter)
4 = init personal parameters, followed by 6 bytes (int: confidence, honesty, desire)
5 = request parameters, followed by zero bytes

6 = send ack, followed by 1 byte (resend check byte)
7 = send all parameters, followed by 6+(7*nodeCount) bytes
8 = exchange about self, followed by 2 bytes
9 = reply to exchange about self, followed by 2 bytes
10 = exchange about others, followed by 1+2 bytes (other ID + data)
11 = reply to exchange about others, followed by 1+2 bytes
12 = proposal, followed by 1 byte (0=friend, 1=enemy)
13 = proposal reply, followed by 2 bytes (0=friend, 1=enemy + 0=accept, 1=decline)

*/

// manually set node count and polling interval here
#define nodeCount 2
#define pollInterval 10000

// pins
#define irTx 9
#define irRx 2

// for IR protocol
#define bitthreshold 1500
#define headerOn 9000
#define headerOff 4500
#define pulseLength 562
#define zeroBit 562
#define oneBit 1687
#define txOn TCCR1A = (1<<COM1A0)
#define txOff TCCR1A = 0

// message types
#define typeAckRequest 0
#define typeSetCount 1
#define typeBegin 2
#define typeReset 3
#define typeInit 4
#define typeRequestPars 5
#define typeSendAck 6
#define typeSendPars 7
#define typeExchangeSelf 8
#define typeExchangeSelfReply 9
#define typeExchangeOther 10
#define typeExchangeOtherReply 11
#define typeProposal 12
#define typeProposalReply 13

// also
#define ackReceived 0
#define ackWrong 1
#define ackMissing 2
#define ackTimeout 3

// some numbers
#define monitorID 255
#define maxNodeCount 10
#define baseIgnoreChance 10
#define baseFalseChance 5
#define baseSilentChance 10
#define tempDiffForStatus 1
#define lightDiffForStatus 1
#define tempLieThreshold 3
#define friendProposalThreshold 10
#define enemyProposalThreshold -10
#define sensorInterval 250

// IR variables
volatile char irStartReceive;
unsigned long irStartTime, irStopTime, irTimeout;
uint8_t irInputBuffer[maxNodeCount+1][10];
uint8_t irBufferIndex;

// monitor variables
unsigned long timeToPoll;
uint8_t receivedPars;

/////////////////////////////////////////////////////////////////////////////
// These are used just for emulating a node
/////////////////////////////////////////////////////////////////////////////
uint8_t myID;
// personal parameters
int confidence, honesty, desire;
// other node parameters
int affection[maxNodeCount], trust[maxNodeCount], status[maxNodeCount];
uint8_t relationship[maxNodeCount];

// other variables
uint8_t temperature, light, waitingForReply, waitingID;
uint8_t othersTemp[maxNodeCount], othersLight[maxNodeCount];
unsigned long myTurnToSpeak, timeToReadSensors, waitingTimer;
/////////////////////////////////////////////////////////////////////////////

ISR(INT0_vect){
  irStartReceive = true;
}

void setup(){
  Serial.begin(38400);
  
  // IR
  irStartReceive = false;
  irBufferIndex = 0;
  pinMode(irRx, INPUT_PULLUP);
  pinMode(irTx, OUTPUT);
  digitalWrite(irTx, LOW);
  EICRA = (1<<ISC01);
  EIMSK = (1<<INT0);
  sei();
  
  // 38kHz carrier, use timer 1A CTC with top 210
  //TCCR1A = (1<<COM1A0) // this turns the output on
  TCCR1A = 0; // this turns it off
  TCCR1B = (1<<WGM12)|(1<<CS10);
  OCR1A = 210;
  
  // establish connection
  while(Serial.available() < 1){
    Serial.write('M');
    delay(200);
  }
  Serial.read();
  
  // request ack from each node
  // to check communications
  getAllAcks();
  
  // send nodeCount
  sendNodeCount();
  
  // send begin signal
  sendBegin();
  
  timeToPoll = millis() + pollInterval;
  receivedPars = false;
}

void loop(){
  // poll parameters every xx seconds
  // wait for a break in conversation
  // to do
  
  // for debug
  //request ack till I get something damnit
  
  myID = 1;
  myTurnToSpeak = millis() + 1000 + 1000*myID;
  while(true){
    debugLoop();
  }
}

// emulates a node and sends info to compy
void debugLoop(){
  uint8_t receivedSomething = false;
  // always be listening for incoming transmissions
  if(irStartReceive){
    if(irReceive() == 1){
      receivedSomething = true;
      delay(1);
      processAndReply();
    }
    irStartReceive = false;
  }
  
  if(receivedSomething){
    if(receivedPars){
      Serial.println("me");
      Serial.print(confidence);
      Serial.print('\t');
      Serial.print(honesty);
      Serial.print('\t');
      Serial.print(desire);
      Serial.print('\t');
      Serial.print(affection[0]);
      Serial.print('\t');
      Serial.print(trust[0]);
      Serial.print('\t');
      Serial.print(status[0]);
      Serial.print('\t');
      Serial.println(relationship[0]);
      Serial.print("Polled ");
      Serial.println(0);
      for(uint8_t i=0; i<9; i++){
        Serial.print(irInputBuffer[0][i]);
        Serial.print('\t');
      }
      Serial.println("");
      for(uint8_t i=1; i<nodeCount+1; i++){
        for(uint8_t j=0; j<10; j++){
          Serial.print(irInputBuffer[i][j]);
          Serial.print('\t');
        }
        Serial.println("");
      }
      receivedPars = false;
    }else{
      Serial.println("Received: ");
      for(uint8_t i=0; i<irBufferIndex; i++){
        Serial.print(irInputBuffer[0][i]);
        Serial.print('\t');
      }
      Serial.println("");
    }
  }
  
  if(timeToPoll < millis()){
    if(myTurnToSpeak < millis()){
      irSendHeader();
      irSendByte(0);
      irSendByte(255);
      irSendByte(typeRequestPars);
      irSendStopBit();
      timeToPoll += pollInterval;
      myTurnToSpeak += 1000*nodeCount;
    }
  }
  
  // This delay allows everyone to have a turn to speak if they want.
  // There is roughly a 1 second delay between each interaction.
  // The turn order is based on node ID.
  if(myTurnToSpeak < millis()){
    uint8_t message = whatToSay();
    uint8_t partener = whoToTalkTo(message);
    startConversation(message, partener);
    // then go to the back of the line
    myTurnToSpeak += 1000*nodeCount;
    
    // If there is no reply within 100ms, give up
    if(waitingForReply){
      if(millis()-waitingTimer > 100){
        waitingForReply = false;
        affection[waitingID]--;
      }
    }
    
    Serial.println("Sent: ");
    Serial.print(message);
    Serial.print('\t');
    Serial.println(partener);
  }
  
  // periodically read the sensors
  if(millis() > timeToReadSensors){
    timeToReadSensors += sensorInterval;
    temperature = 100;
    light = 127;
    modifySelfParameters();
  }
}

void getAllAcks(){
  uint8_t tmp;
  for(uint8_t i=0; i<nodeCount; i++){
    delay(10);
    tmp = getAck(i);
    if(i == 1){
      tmp = ackReceived;
    }
    if(tmp == ackTimeout){
      Serial.println("ack timeout from ");
      Serial.println(i);
      i--;
    }
    if(tmp == ackMissing){
      Serial.println("no ack from ");
      Serial.println(i);
      i--;
    }
    if(tmp == ackWrong){
      Serial.println("wrong ack from ");
      Serial.println(i);
      i--;
    }if(tmp == ackReceived){
      Serial.print("ack from ");
      Serial.println(i);
    }
  }
}

uint8_t getAck(uint8_t id){
  // send request
  irSendHeader();
  irSendByte(id);
  irSendByte(255);
  irSendByte(typeAckRequest);
  irSendByte(0xAC);
  irSendStopBit();
  
  // wait for response
  irStartReceive = false;
  irTimeout = millis()+200;
  while(!irStartReceive){
    if(millis() > irTimeout){
      // no response
      return ackTimeout;
    }
  }
  if(irReceive() == 1){
    if(irInputBuffer[0][3] == 0xAC){
      return ackReceived;
    }else{
      return ackWrong;
    }
  }
  return ackMissing;
}

void sendNodeCount(){
  irSendHeader();
  irSendByte(255);
  irSendByte(255);
  irSendByte(typeSetCount);
  irSendByte(nodeCount);
  irSendStopBit();
}

void sendBegin(){
  irSendHeader();
  irSendByte(255);
  irSendByte(255);
  irSendByte(typeBegin);
  irSendStopBit();
}

/////////////////////////////////////////////////////////////////////////////
// for node emulation
/////////////////////////////////////////////////////////////////////////////
// Decide what type of message to send
uint8_t whatToSay(){
  int8_t tmp;
  // There is a chance to say nothing
  if(desire > baseSilentChance){
    tmp = 0;
  }else{
    tmp = baseSilentChance - desire;
  }
  if((micros()%100) > tmp){
    // not silent, decide what to say
    tmp = 127;
    for(uint8_t i = 0; i<nodeCount; i++){
      if((affection[i] > friendProposalThreshold) && (relationship[i] != 1)){
        tmp = i;
      }else if((affection[i] < enemyProposalThreshold) && (relationship[i] != 2)){
        tmp = -1-i;
      }
    }
    if((tmp < 127)&&(((micros()>>1)%100) < 50)){
      // propose
      if(tmp >= 0){
        return 3; // friend
      }else{
        return 4; // enemy
      }
    }else{
      if(nodeCount < 3){
        return 1;
      }
      if(((micros()>>2)%100) < (50+2*confidence)){
        return 1; // exchange about self
      }else{
        return 2; // exchange about other
      }
    }
  }
  return 255;
}

// Decide who to talk to
// message = 255 silent, 1 exchange self, 2 exchange other, 3 propose friend, 4 propose enemy
uint8_t whoToTalkTo(uint8_t message){
  if(message == 255){
    return 0;
  }
  if(message == 1){
    // exchange about self
    uint8_t tmp = nodeCount;
    for(uint8_t i = 0; i<nodeCount; i++){
      if(affection[i] > 0){
        tmp += affection[i];
      }
    }
    uint8_t rand = micros()%tmp;
    tmp = 0;
    for(uint8_t i = 0; i<nodeCount; i++){
      tmp++;
      if(affection[i] > 0){
        tmp += affection[i];
      }
      if(rand < tmp){
        if(i == myID){
          return (i+1)%nodeCount;
        }else{
          return i;
        }
      }
    }
  }
  if(message == 2){
    // exchange about other
    uint8_t tmp = nodeCount;
    for(uint8_t i = 0; i<nodeCount; i++){
      if(trust[i] < 0){
        tmp -= trust[i];
      }
    }
    uint8_t rand = micros()%tmp;
    tmp = 0;
    for(uint8_t i = 0; i<nodeCount; i++){
      tmp++;
      if(trust[i] < 0){
        tmp -= trust[i];
      }
      if(rand < tmp){
        if(i == myID){
          return (i+1)%nodeCount;
        }else{
          return i;
        }
      }
    }
  }
  if(message == 3){
    // friend proposal
    uint8_t tmp = 0;
    for(uint8_t i = 0; i<nodeCount; i++){
      if((affection[i] > friendProposalThreshold) && (relationship[i] != 1)){
        tmp = i;
      }
    }
    return tmp;
  }
  if(message == 4){
    // enemy proposal
    uint8_t tmp = 0;
    for(uint8_t i = 0; i<nodeCount; i++){
      if((affection[i] < enemyProposalThreshold) && (relationship[i] != 2)){
        tmp = i;
      }
    }
    return tmp;
  }
  // if something unexpected happened
  return (myID+1)%nodeCount;
}

void startConversation(uint8_t message, uint8_t partener){
  if(message == 255){
    return;
  }else{
    waitingForReply = true;
    waitingID = partener;
    waitingTimer = millis();
  }
  if(message == 1){
    int chanceToLie = baseFalseChance - honesty - trust[partener]/2;
    irSendHeader();
    irSendByte(partener);
    irSendByte(myID);
    irSendByte(typeExchangeSelf);
    if(chanceToLie > 0){
      if((micros()%100) < chanceToLie){
        // send false data
        // temp is lower by lie threshold + chanceToLie
        // light is higher by chanceToLie
        if(temperature > tempLieThreshold + chanceToLie){
          irSendByte(temperature - tempLieThreshold - chanceToLie);
        }else{
          irSendByte(0);
        }
        irSendByte(light + chanceToLie);
      }else{
        irSendByte(temperature);
        irSendByte(light);
      }
    }else{
      irSendByte(temperature);
      irSendByte(light);
    }
    irSendStopBit();
  }else if(message == 2){
    // choose someone to talk to
    uint8_t target = (micros()>>2)%nodeCount;
    if(target == myID){
      target = (target+1)%nodeCount;
    }
    if(target == partener){
      target = (target+1)%nodeCount;
    }
    if(target == myID){
      target = (target+1)%nodeCount;
    }
    irSendHeader();
    irSendByte(target);
    irSendByte(myID);
    irSendByte(typeExchangeOther);
    irSendByte(partener);
    // There is a chance to lie if the other is an enemy
    int chanceToLie = 0;
    if((othersTemp[partener] > tempLieThreshold) && (relationship[partener] == 2)){
      chanceToLie = baseFalseChance - honesty - affection[partener]/2;
    }
    if(((micros()>>2)%100) < chanceToLie){
      irSendByte(othersTemp[partener] - tempLieThreshold);
      irSendByte(othersLight[partener] + tempLieThreshold);
    }else{
      irSendByte(othersTemp[partener]);
      irSendByte(othersLight[partener]);
    }
    irSendStopBit();
  }else if(message == 3){
    irSendHeader();
    irSendByte(partener);
    irSendByte(myID);
    irSendByte(typeProposal);
    irSendByte(0);
    irSendStopBit();
  }else if(message == 4){
    irSendHeader();
    irSendByte(partener);
    irSendByte(myID);
    irSendByte(typeProposal);
    irSendByte(1);
    irSendStopBit();
  }
}

// Sensor readings affect personal parameters
void modifySelfParameters(){
  int averageTemp = temperature;
  int averageLight = light;
  uint8_t tmp = 1;
  for(uint8_t i = 0; i<nodeCount; i++){
    if(othersTemp[i] > 0){
      averageTemp += othersTemp[i];
      averageLight += othersLight[i];
      tmp++;
    }
  }
  averageTemp = averageTemp/tmp;
  averageLight = averageLight/tmp;
  // confidence is influenced by light
  confidence = light - averageLight;
  // honesty is influenced by temperature
  honesty = averageTemp - temperature;
}

/*
Here lies all of the important incoming communication handling and social math.
Good luck.
*/
void processAndReply(){
  // These two types don't depend on destination
  if(irInputBuffer[0][2] == typeSetCount){
    // nodeCount = irInputBuffer[0][3];
    return;
  }else if(irInputBuffer[0][2] == typeReset){
    for(uint8_t i = 0; i<nodeCount; i++){
      affection[i] = 0;
      trust[i] = 0;
      status[i] = 0;
      relationship[i] = 0;
    }
    return;
  }
  
  // these two are only for the monitor
  if(irInputBuffer[0][2] == typeSendAck){
    // handled elsewhere
  }else if(irInputBuffer[0][2] == typeSendPars){
    receivedPars = true;
  }
  
  // if the destination is not this node, do nothing
  if(irInputBuffer[0][0] != myID){
    return;
  }
  
  // handle each different type
  if(irInputBuffer[0][2] == typeAckRequest){
    irSendHeader();
    irSendByte(monitorID);
    irSendByte(myID);
    irSendByte(typeSendAck);
    irSendByte(irInputBuffer[0][3]);
    irSendStopBit();
  }else if(irInputBuffer[0][2] == typeInit){
    confidence = (irInputBuffer[0][3] << 8)|irInputBuffer[0][4];
    honesty = (irInputBuffer[0][5] << 8)|irInputBuffer[0][6];
    desire = (irInputBuffer[0][7] << 8)|irInputBuffer[0][8];
  }else if(irInputBuffer[0][2] == typeRequestPars){
    irSendHeader();
    irSendByte(monitorID);
    irSendByte(myID);
    irSendByte(typeSendPars);
    irSendByte((uint16_t)confidence >> 8);
    irSendByte((uint16_t)confidence & 0xFF);
    irSendByte((uint16_t)honesty >> 8);
    irSendByte((uint16_t)honesty & 0xFF);
    irSendByte((uint16_t)desire >> 8);
    irSendByte((uint16_t)desire & 0xFF);
    for(uint8_t i = 0; i<nodeCount; i++){
      irSendByte((uint16_t)affection[i] >> 8);
      irSendByte((uint16_t)affection[i] & 0xFF);
      irSendByte((uint16_t)trust[i] >> 8);
      irSendByte((uint16_t)trust[i] & 0xFF);
      irSendByte((uint16_t)status[i] >> 8);
      irSendByte((uint16_t)status[i] & 0xFF);
      irSendByte(relationship[i]);
    }
    irSendStopBit();
  }else if(irInputBuffer[0][2] == typeExchangeSelf){
    int chanceToIgnore = baseIgnoreChance - desire - affection[irInputBuffer[0][1]] - status[irInputBuffer[0][1]];
    int chanceToLie = baseFalseChance - honesty - trust[irInputBuffer[0][1]]/2;
    if(chanceToIgnore > 0){
      if((micros()%100) < chanceToIgnore){
        // ignore
        return;
      }
    }
    irSendHeader();
    irSendByte(irInputBuffer[0][1]);
    irSendByte(myID);
    irSendByte(typeExchangeSelfReply);
    if(chanceToLie > 0){
      if(((micros()>>2)%100) < chanceToLie){
        // send false data
        // temp is lower by lie threshold + chanceToLie
        // light is higher by chanceToLie
        if(temperature > tempLieThreshold + chanceToLie){
          irSendByte(temperature - tempLieThreshold - chanceToLie);
        }else{
          irSendByte(0);
        }
        irSendByte(light + chanceToLie);
      }else{
        irSendByte(temperature);
        irSendByte(light);
      }
    }else{
      irSendByte(temperature);
      irSendByte(light);
    }
    irSendStopBit();
    
    // then look at their data
    othersTemp[irInputBuffer[0][1]] = irInputBuffer[0][3];
    othersLight[irInputBuffer[0][1]] = irInputBuffer[0][4];
    
    // their data affects their status
    int averageTemp = temperature;
    int averageLight = light;
    uint8_t tmp = 1;
    for(uint8_t i = 0; i<nodeCount; i++){
      if(othersTemp[i] > 0){
        averageTemp += othersTemp[i];
        averageLight += othersLight[i];
        tmp++;
      }
    }
    averageTemp = averageTemp/tmp;
    averageLight = averageLight/tmp;
    if((irInputBuffer[0][3] < averageTemp - tempDiffForStatus) && (irInputBuffer[0][4] > averageLight + lightDiffForStatus)){
      status[irInputBuffer[0][1]]++;
    }else if((irInputBuffer[0][3] > averageTemp + tempDiffForStatus) && (irInputBuffer[0][4] < averageLight - lightDiffForStatus)){
      status[irInputBuffer[0][1]]--;
    }
    
  }else if(irInputBuffer[0][2] == typeExchangeSelfReply){
    if(!waitingForReply){
      // huh?
      return;
    }
    waitingForReply = false;
    affection[irInputBuffer[0][1]]++;
    othersTemp[irInputBuffer[0][1]] = irInputBuffer[0][3];
    othersLight[irInputBuffer[0][1]] = irInputBuffer[0][4];
    
    // their data affects their status
    int averageTemp = temperature;
    int averageLight = light;
    uint8_t tmp = 1;
    for(uint8_t i = 0; i<nodeCount; i++){
      if(othersTemp[i] > 0){
        averageTemp += othersTemp[i];
        averageLight += othersLight[i];
        tmp++;
      }
    }
    averageTemp = averageTemp/tmp;
    averageLight = averageLight/tmp;
    if((irInputBuffer[0][3] < averageTemp - tempDiffForStatus) && (irInputBuffer[0][4] > averageLight + lightDiffForStatus)){
      status[irInputBuffer[0][1]]++;
    }else if((irInputBuffer[0][3] > averageTemp + tempDiffForStatus) && (irInputBuffer[0][4] < averageLight - lightDiffForStatus)){
      status[irInputBuffer[0][1]]--;
    }
    
  }else if(irInputBuffer[0][2] == typeExchangeOther){
    int chanceToIgnore = baseIgnoreChance - desire - affection[irInputBuffer[0][1]] - status[irInputBuffer[0][1]];
    if(chanceToIgnore > 0){
      if((micros()%100) < chanceToIgnore){
        // ignore
        return;
      }
    }
    // first send the data
    // if the other is an enemy, there is a chance to lie
    irSendHeader();
    irSendByte(irInputBuffer[0][1]);
    irSendByte(myID);
    irSendByte(typeExchangeOtherReply);
    irSendByte(irInputBuffer[0][3]);
    // just reuse the ignore variable for efficiency
    chanceToIgnore = 0;
    if((othersTemp[irInputBuffer[0][3]] > tempLieThreshold) && (relationship[irInputBuffer[0][3]] == 2)){
      chanceToIgnore = baseFalseChance - honesty - affection[irInputBuffer[0][3]]/2;
    }
    if(((micros()>>2)%100) < chanceToIgnore){
      irSendByte(othersTemp[irInputBuffer[0][3]] - tempLieThreshold);
      irSendByte(othersLight[irInputBuffer[0][3]] + tempLieThreshold);
    }else{
      irSendByte(othersTemp[irInputBuffer[0][3]]);
      irSendByte(othersLight[irInputBuffer[0][3]]);
    }
    irSendStopBit();
    
    // then take a look at incoming data
    if(irInputBuffer[0][4] == 0){
      // they had no data
      return;
    }
    if(othersTemp[irInputBuffer[0][3]] == 0){
      // I had no data
      othersTemp[irInputBuffer[0][3]] = irInputBuffer[0][4];
      othersLight[irInputBuffer[0][3]] = irInputBuffer[0][5];
      return;
    }
    // We both had data. time to check for lies
    if(othersTemp[irInputBuffer[0][3]] > irInputBuffer[0][4]){
      if(othersTemp[irInputBuffer[0][3]] - irInputBuffer[0][4] > tempLieThreshold){
        trust[irInputBuffer[0][3]]--;
        affection[irInputBuffer[0][3]]--;
      }
    }else if(irInputBuffer[0][4] - othersTemp[irInputBuffer[0][3]] > tempLieThreshold){
        trust[irInputBuffer[0][3]]--;
        affection[irInputBuffer[0][3]]--;
        othersTemp[irInputBuffer[0][3]] = irInputBuffer[0][4];
        othersLight[irInputBuffer[0][3]] = irInputBuffer[0][5];
    }
    
  }else if(irInputBuffer[0][2] == typeExchangeOtherReply){
    if(!waitingForReply){
      // huh?
      return;
    }
    waitingForReply = false;
    if(irInputBuffer[0][4] == 0){
      // they had no data
      return;
    }
    if(othersTemp[irInputBuffer[0][3]] == 0){
      // I had no data
      othersTemp[irInputBuffer[0][3]] = irInputBuffer[0][4];
      othersLight[irInputBuffer[0][3]] = irInputBuffer[0][5];
      return;
    }
    // We both had data. time to check for lies
    if(othersTemp[irInputBuffer[0][3]] > irInputBuffer[0][4]){
      if(othersTemp[irInputBuffer[0][3]] - irInputBuffer[0][4] > tempLieThreshold){
        trust[irInputBuffer[0][3]]--;
        affection[irInputBuffer[0][3]]--;
      }
    }else if(irInputBuffer[0][4] - othersTemp[irInputBuffer[0][3]] > tempLieThreshold){
        trust[irInputBuffer[0][3]]--;
        affection[irInputBuffer[0][3]]--;
        othersTemp[irInputBuffer[0][3]] = irInputBuffer[0][4];
        othersLight[irInputBuffer[0][3]] = irInputBuffer[0][5];
    }
    
  }else if(irInputBuffer[0][2] == typeProposal){
    irSendHeader();
    irSendByte(irInputBuffer[0][1]);
    irSendByte(myID);
    irSendByte(typeProposalReply);
    irSendByte(irInputBuffer[0][3]);
    
    if(irInputBuffer[0][3] == 0){
      // friendship
      if(relationship[irInputBuffer[0][1]] == 2){
        // What happened? I thought we were enemies.
        relationship[irInputBuffer[0][1]] = 0;
        affection[irInputBuffer[0][1]] = 0;
        // decline
        irSendByte(1);
      }else if(affection[irInputBuffer[0][1]] > friendProposalThreshold){
        // I like you too.
        relationship[irInputBuffer[0][1]] = 1;
        // accept
        irSendByte(0);
      }else{
        // Nope. I don't think we're ready for that type of relationship.
        // decline
        irSendByte(1);
      }
    }else if(irInputBuffer[0][3] == 1){
      // enemies
      if(relationship[irInputBuffer[0][1]] == 1){
        // What happened? We were such good friends.
        relationship[irInputBuffer[0][1]] = 0;
        affection[irInputBuffer[0][1]] = 0;
        // decline
        irSendByte(1);
      }else if(affection[irInputBuffer[0][1]] < enemyProposalThreshold){
        // I don't like you either.
        relationship[irInputBuffer[0][1]] = 2;
        // accept
        irSendByte(0);
      }else{
        // Nope. I don't think we're ready for that type of relationship.
        // decline
        irSendByte(1);
      }
    }
    irSendStopBit();
    
  }else if(irInputBuffer[0][2] == typeProposalReply){
    if(!waitingForReply){
      // huh?
      return;
    }
    waitingForReply = false;
    if(irInputBuffer[0][3] == 0){
      if(irInputBuffer[0][4] == 0){
        // made a friend
        relationship[irInputBuffer[0][1]] = 1;
      }else if(irInputBuffer[0][4] == 1){
        // no friend
        relationship[irInputBuffer[0][1]] = 0;
        affection[irInputBuffer[0][1]]--;
      }
    }else if(irInputBuffer[0][3] == 0){
      if(irInputBuffer[0][4] == 0){
        // made an enemy
        relationship[irInputBuffer[0][1]] = 2;
      }else if(irInputBuffer[0][4] == 1){
        // no enemy
        relationship[irInputBuffer[0][1]] = 0;
        affection[irInputBuffer[0][1]]++;
      }
    }
  }
}
/////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
// Here lies all of the IR communication stuff
//////////////////////////////////////////////////////////////////////////////////////////////
// receives IR input
// returns 0 = no input, 1 = received,
// 2 = noise, 3 = no rise(stuck)
// 4 = long header(bad timing), 5 = incomplete data
uint8_t irReceive(){
  uint8_t bytesRemaining = 0;
  // check for incoming signal
  // start time has already been set
  // wait for 200us to see if it was a bit of noise
  delayMicroseconds(200);
  if((PIND&4) > 0){
    // it was noise
    irStartReceive = false;
    return 2;
  }
  
  // check the start interval length
  irTimeout = millis();
  while(((PIND&4) == 0)){ // wait for rise
    if(millis()-irTimeout > 20){
      irStartReceive = false;
      return 3;
    }
  }
  irTimeout = millis();
  while(((PIND&4) > 0)){ // wait for fall
    if(millis()-irTimeout > 20){
      irStartReceive = false;
      return 4;
    }
  }
  irStartTime = micros();
  
  // at this point a successful header was received
  // get the first three bytes
  irBufferIndex = 0;
  if(!(irReceiveByte() && irReceiveByte() && irReceiveByte())){
    return 5;
  }
  
  // check message type to determine length
  if(irInputBuffer[0][2] == typeAckRequest){
    bytesRemaining = 1;
  }else if(irInputBuffer[0][2] == typeSetCount){
    bytesRemaining = 1;
  }else if(irInputBuffer[0][2] == typeReset){
    bytesRemaining = 0;
  }else if(irInputBuffer[0][2] == typeInit){
    bytesRemaining = 6;
  }else if(irInputBuffer[0][2] == typeRequestPars){
    bytesRemaining = 0;
  }else if(irInputBuffer[0][2] == typeSendAck){
    bytesRemaining = 1;
  }else if(irInputBuffer[0][2] == typeSendPars){
    // this is special
    bytesRemaining = 0;
    uint8_t i, j;
    for(i=3; i<9; i++){
      if(!irReceiveByte(0, i)){
        return 5;
      }
    }
    for(i=1; i<nodeCount+1; i++){
      for(j=3; j<10; j++){
        if(!irReceiveByte(i, j)){
          return 5;
        }
      }
    }
  }else if(irInputBuffer[0][2] == typeExchangeSelf){
    bytesRemaining = 2;
  }else if(irInputBuffer[0][2] == typeExchangeSelfReply){
    bytesRemaining = 2;
  }else if(irInputBuffer[0][2] == typeExchangeOther){
    bytesRemaining = 3;
  }else if(irInputBuffer[0][2] == typeExchangeOtherReply){
    bytesRemaining = 3;
  }else if(irInputBuffer[0][2] == typeProposal){
    bytesRemaining = 1;
  }else if(irInputBuffer[0][2] == typeProposalReply){
    bytesRemaining = 2;
  }
  
  // receive all remaining bytes
  while(bytesRemaining > 0){
    if(!irReceiveByte()){
      return 5;
    }
    bytesRemaining--;
  }
  
  irWaitForStopBit();
  irStartReceive = false;
  return 1;
}

bool irReceiveByte(){
  uint8_t b = 0;
  uint8_t i = 0;
  while(i<8){
    i++;
    b <<= 1;
    irTimeout = millis();
    while(((PIND&4) == 0)){ // wait for rise
      if(millis()-irTimeout > 20){
        return false;
      }
    }
    irTimeout = millis();
    while(((PIND&4) > 0)){ // wait for fall
      if(millis()-irTimeout > 20){
        return false;
      }
    }
    irStopTime = micros();
    if(irStopTime-irStartTime > bitthreshold){
      b++;
    }
    irStartTime = irStopTime;
  }
  
  if(irBufferIndex < 10){
    irInputBuffer[0][irBufferIndex] = b;
    irBufferIndex = irBufferIndex+1;
  }
  // else ignore it
  
  return true;
}

bool irReceiveByte(uint8_t nodeIndex, uint8_t parIndex){
  uint8_t b = 0;
  uint8_t i = 0;
  while(i<8){
    i++;
    b <<= 1;
    irTimeout = millis();
    while(((PIND&4) == 0)){ // wait for rise
      if(millis()-irTimeout > 20){
        return false;
      }
    }
    irTimeout = millis();
    while(((PIND&4) > 0)){ // wait for fall
      if(millis()-irTimeout > 20){
        return false;
      }
    }
    irStopTime = micros();
    if(irStopTime-irStartTime > bitthreshold){
      b++;
    }
    irStartTime = irStopTime;
  }
  
  if((nodeIndex < maxNodeCount) && (parIndex < 10)){
    irInputBuffer[nodeIndex][parIndex] = b;
  }
  // else ignore it
  
  return true;
}

void irWaitForStopBit(){
  irTimeout = millis();
  while(((PIND&4) == 0)){ // wait for rise
    if(millis()-irTimeout > 20){
      return;
    }
  }
}

void irSendHeader(){
  // header
  txOn;
  delayMicroseconds(headerOn);
  txOff;
  digitalWrite(irTx, LOW);
  delayMicroseconds(headerOff);
}

void irSendByte(uint8_t output){
  for(int8_t i=7; i>=0; i--){
    if(output & (1<<i)){
      txOn;
      delayMicroseconds(pulseLength);
      txOff;
      digitalWrite(irTx, LOW);
      delayMicroseconds(oneBit);
    }else{
      txOn;
      delayMicroseconds(pulseLength);
      txOff;
      digitalWrite(irTx, LOW);
      delayMicroseconds(zeroBit);
    }
  }
}

void irSendStopBit(){
  // stop bit
  txOn;
  delayMicroseconds(pulseLength);
  txOff;
  digitalWrite(irTx, LOW);
  delay(1);
}
