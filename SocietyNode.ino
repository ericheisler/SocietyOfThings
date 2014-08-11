
/*
This is the node.

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

*Note: During an exchange, nodes have a chance to send false data or
       to ignore the exchange(not reply).
Parameters include:
about self - confidence, honesty, desire to socialize
about others - affection, trust, percieved social status, relationship

Parameters affect conversation in the following ways.
confidence: higher values mean more exchanges are about self and more chance to propose
honesty: lower values mean higher chance and magnitude of sending false self data
desire: higher values mean more frequent exchanges and lower chance to ignore

affection: more frequent exchanges and less chance to ignore nodes with high affection
           When affection is high enough, there's a chance to propose/accept friendship.
           enemy proposals for low values
trust: lower values mean more frequent exchanges with 3rd parties about this node
       higher values mean lower chance of sending false data to this node
status: high values mean lower chance to ignore exchanges from high status node

Parameters are affected by conversation in the following ways.
affection: increased by replies to self exchanges and when enemy proposal is rejected
           decreased by ignored exchanges, upon discovering false data 
           and when friendship proposal is rejected
trust: increased by verifying true data with 3rd party exchanges
       decreased by discovering false data as above
status: increased by having better environmental parameters
        decreased by opposite

Parameters are affected by environmental input in the following ways.
confidence: Influenced by light intensity (wealth)
honesty: Influenced by temperature (comfort)
desire: still thinking about this

Environmental parameters are temperature and light intensity.
Ideal temp is low, ideal light is high

*/

// unique to each device
#define myID 0

// pins
#define txpin 1
#define rxpin 2
#define temppin 4
#define temppinA 2
#define lightpin 3
#define lightpinA 3

// for IR protocol
#define bitthreshold 1500
#define headerOn 9000
#define headerOff 4500
#define pulseLength 562
#define zeroBit 562
#define oneBit 1687
#define txOn TCCR1 = (1<<CTC1)|(1<<COM1A0)|(1<<CS10)
#define txOff TCCR1 = (1<<CTC1)|(1<<CS10)

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
uint8_t irInputBuffer[10];
uint8_t irBufferIndex;

// personal parameters
int confidence, honesty, desire;
// other node parameters
int affection[maxNodeCount], trust[maxNodeCount], status[maxNodeCount];
uint8_t relationship[maxNodeCount];

// other variables
uint8_t nodeCount, temperature, light, waitingForReply, waitingID;
uint8_t othersTemp[maxNodeCount], othersLight[maxNodeCount];
unsigned long myTurnToSpeak, timeToReadSensors, waitingTimer;

ISR(INT0_vect){
  irStartReceive = true;
}

void setup(){
  // IR
  irStartReceive = false;
  irBufferIndex = 0;
  pinMode(rxpin, INPUT_PULLUP);
  pinMode(txpin, OUTPUT);
  digitalWrite(txpin, LOW);
  MCUCR |= (1<<ISC01);
  GIMSK |= (1<<INT0);
  sei();
  // 38kHz carrier, use CTC with top 104
  //TCCR1 = (1<<CTC1)|(1<<COM1A0)|(1<<CS10); // this turns the output on
  TCCR1 = (1<<CTC1)|(1<<CS10); // this turns it off
  OCR1C = 104;
  
  // init vars
  confidence = 0;
  honesty = 0;
  desire = 0;
  waitingForReply = false;
  
  // wait for ack request from monitor
  char waitingForBegin = true;
  while(waitingForBegin){
    if(irStartReceive){
      if(irReceive() == 1){
        if((irInputBuffer[2] == typeAckRequest) && (irInputBuffer[0] == myID)){
          // reply 
          delay(1);
          irSendHeader();
          irSendByte(monitorID);
          irSendByte(myID);
          irSendByte(typeSendAck);
          irSendByte(irInputBuffer[3]);
          irSendStopBit();
          waitingForBegin = false;
        }
        irStartReceive = false;
      }
    }
  }
  
  // wait for node count from monitor
  waitingForBegin = true;
  while(waitingForBegin){
    if(irStartReceive){
      if(irReceive() == 1){
        if(irInputBuffer[2] == typeSetCount){
          nodeCount = irInputBuffer[3];
          waitingForBegin = false;
        }
        irStartReceive = false;
      }
    }
  }
  
  // wait for a begin signal from the monitor
  waitingForBegin = true;
  while(waitingForBegin){
    if(irStartReceive){
      if(irReceive() == 1){
        if(irInputBuffer[2] == typeBegin){
          waitingForBegin = false;
        }
        irStartReceive = false;
      }
    }
  }
  myTurnToSpeak = millis() + 1000 + 1000*myID;
  
  timeToReadSensors = millis();
}

void loop(){
  // always be listening for incoming transmissions
  if(irStartReceive){
    if(irReceive() == 1){
      delay(1);
      processAndReply();
    }
    irStartReceive = false;
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
  }
  
  // If there is no reply within 100ms, give up
  if(waitingForReply){
    if(millis()-waitingTimer > 100){
      waitingForReply = false;
      affection[waitingID]--;
    }
  }
  
  // periodically read the sensors
  // and modify the personal parameters
  if(millis() > timeToReadSensors){
    timeToReadSensors += sensorInterval;
    temperature = analogRead(temppinA)>>2;
    light = analogRead(lightpinA)>>2;
    modifySelfParameters();
  }
}

// Decide what type of message to send
// message = 255 silent, 1 exchange self, 2 exchange other, 3 propose friend, 4 propose enemy
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
    tmp = 0;
    for(uint8_t i = 0; i<nodeCount; i++){
      if((affection[i] > friendProposalThreshold) && (relationship[i] != 1)){
        tmp = 1;
      }else if((affection[i] < enemyProposalThreshold) && (relationship[i] != 2)){
        tmp = -1;
      }
    }
    if((tmp != 0)&&(((micros()>>1)%100) < 50)){
      // propose
      if(tmp > 0){
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
    // Each node has 1+affection tickets. One ticket is chosen at random.
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
    // Each node with (trust < 0)has (1-trust) tickets, 1 otherwise.
    // One ticket is chosen at random.
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
    // choose someone to talk to at random
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
  if(irInputBuffer[2] == typeSetCount){
    // nodeCount = irInputBuffer[3];
    return;
  }else if(irInputBuffer[2] == typeReset){
    for(uint8_t i = 0; i<nodeCount; i++){
      confidence = 0;
      honesty = 0;
      desire = 0;
      affection[i] = 0;
      trust[i] = 0;
      status[i] = 0;
      relationship[i] = 0;
    }
    return;
  }
  
  // if the destination is not this node, do nothing
  if(irInputBuffer[0] != myID){
    return;
  }
  
  // handle each different type
  if(irInputBuffer[2] == typeAckRequest){
    irSendHeader();
    irSendByte(monitorID);
    irSendByte(myID);
    irSendByte(typeSendAck);
    irSendByte(irInputBuffer[3]);
    irSendStopBit();
  }else if(irInputBuffer[2] == typeInit){
    confidence = (irInputBuffer[3] << 8)|irInputBuffer[4];
    honesty = (irInputBuffer[5] << 8)|irInputBuffer[6];
    desire = (irInputBuffer[7] << 8)|irInputBuffer[8];
  }else if(irInputBuffer[2] == typeRequestPars){
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
  }else if(irInputBuffer[2] == typeSendAck){
    // should never get here
  }else if(irInputBuffer[2] == typeSendPars){
    // should never get here
  }else if(irInputBuffer[2] == typeExchangeSelf){
    int chanceToIgnore = baseIgnoreChance - desire - affection[irInputBuffer[1]] - status[irInputBuffer[1]];
    int chanceToLie = baseFalseChance - honesty - trust[irInputBuffer[1]]/2;
    if(chanceToIgnore > 0){
      if((micros()%100) < chanceToIgnore){
        // ignore
        return;
      }
    }
    irSendHeader();
    irSendByte(irInputBuffer[1]);
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
    othersTemp[irInputBuffer[1]] = irInputBuffer[3];
    othersLight[irInputBuffer[1]] = irInputBuffer[4];
    
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
    if((irInputBuffer[3] < averageTemp - tempDiffForStatus) && (irInputBuffer[4] > averageLight + lightDiffForStatus)){
      status[irInputBuffer[1]]++;
    }else if((irInputBuffer[3] > averageTemp + tempDiffForStatus) && (irInputBuffer[4] < averageLight - lightDiffForStatus)){
      status[irInputBuffer[1]]--;
    }
    
  }else if(irInputBuffer[2] == typeExchangeSelfReply){
    if(!waitingForReply){
      // huh?
      return;
    }
    waitingForReply = false;
    affection[irInputBuffer[1]]++;
    othersTemp[irInputBuffer[1]] = irInputBuffer[3];
    othersLight[irInputBuffer[1]] = irInputBuffer[4];
    
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
    if((irInputBuffer[3] < averageTemp - tempDiffForStatus) && (irInputBuffer[4] > averageLight + lightDiffForStatus)){
      status[irInputBuffer[1]]++;
    }else if((irInputBuffer[3] > averageTemp + tempDiffForStatus) && (irInputBuffer[4] < averageLight - lightDiffForStatus)){
      status[irInputBuffer[1]]--;
    }
    
  }else if(irInputBuffer[2] == typeExchangeOther){
    int chanceToIgnore = baseIgnoreChance - desire - affection[irInputBuffer[1]] - status[irInputBuffer[1]];
    if(chanceToIgnore > 0){
      if((micros()%100) < chanceToIgnore){
        // ignore
        return;
      }
    }
    // first send the data
    // if the other is an enemy, there is a chance to lie
    irSendHeader();
    irSendByte(irInputBuffer[1]);
    irSendByte(myID);
    irSendByte(typeExchangeOtherReply);
    irSendByte(irInputBuffer[3]);
    // just reuse the ignore variable for efficiency
    chanceToIgnore = 0;
    if((othersTemp[irInputBuffer[3]] > tempLieThreshold) && (relationship[irInputBuffer[3]] == 2)){
      chanceToIgnore = baseFalseChance - honesty - affection[irInputBuffer[3]]/2;
    }
    if(((micros()>>2)%100) < chanceToIgnore){
      irSendByte(othersTemp[irInputBuffer[3]] - tempLieThreshold);
      irSendByte(othersLight[irInputBuffer[3]] + tempLieThreshold);
    }else{
      irSendByte(othersTemp[irInputBuffer[3]]);
      irSendByte(othersLight[irInputBuffer[3]]);
    }
    irSendStopBit();
    
    // then take a look at incoming data
    if(irInputBuffer[4] == 0){
      // they had no data
      return;
    }
    if(othersTemp[irInputBuffer[3]] == 0){
      // I had no data
      othersTemp[irInputBuffer[3]] = irInputBuffer[4];
      othersLight[irInputBuffer[3]] = irInputBuffer[5];
      return;
    }
    // We both had data. time to check for lies
    if(othersTemp[irInputBuffer[3]] > irInputBuffer[4]){
      if(othersTemp[irInputBuffer[3]] - irInputBuffer[4] > tempLieThreshold){
        trust[irInputBuffer[3]]--;
        affection[irInputBuffer[3]]--;
      }
    }else if(irInputBuffer[4] - othersTemp[irInputBuffer[3]] > tempLieThreshold){
        trust[irInputBuffer[3]]--;
        affection[irInputBuffer[3]]--;
        othersTemp[irInputBuffer[3]] = irInputBuffer[4];
        othersLight[irInputBuffer[3]] = irInputBuffer[5];
    }
    
  }else if(irInputBuffer[2] == typeExchangeOtherReply){
    if(!waitingForReply){
      // huh?
      return;
    }
    waitingForReply = false;
    if(irInputBuffer[4] == 0){
      // they had no data
      return;
    }
    if(othersTemp[irInputBuffer[3]] == 0){
      // I had no data
      othersTemp[irInputBuffer[3]] = irInputBuffer[4];
      othersLight[irInputBuffer[3]] = irInputBuffer[5];
      return;
    }
    // We both had data. time to check for lies
    if(othersTemp[irInputBuffer[3]] > irInputBuffer[4]){
      if(othersTemp[irInputBuffer[3]] - irInputBuffer[4] > tempLieThreshold){
        trust[irInputBuffer[3]]--;
        affection[irInputBuffer[3]]--;
      }
    }else if(irInputBuffer[4] - othersTemp[irInputBuffer[3]] > tempLieThreshold){
        trust[irInputBuffer[3]]--;
        affection[irInputBuffer[3]]--;
        othersTemp[irInputBuffer[3]] = irInputBuffer[4];
        othersLight[irInputBuffer[3]] = irInputBuffer[5];
    }
    
  }else if(irInputBuffer[2] == typeProposal){
    irSendHeader();
    irSendByte(irInputBuffer[1]);
    irSendByte(myID);
    irSendByte(typeProposalReply);
    irSendByte(irInputBuffer[3]);
    
    if(irInputBuffer[3] == 0){
      // friendship
      if(relationship[irInputBuffer[1]] == 2){
        // What happened? I thought we were enemies.
        relationship[irInputBuffer[1]] = 0;
        affection[irInputBuffer[1]] = 0;
        // decline
        irSendByte(1);
      }else if(affection[irInputBuffer[1]] > friendProposalThreshold){
        // I like you too.
        relationship[irInputBuffer[1]] = 1;
        // accept
        irSendByte(0);
      }else{
        // Nope. I don't think we're ready for that type of relationship.
        // decline
        irSendByte(1);
      }
    }else if(irInputBuffer[3] == 1){
      // enemies
      if(relationship[irInputBuffer[1]] == 1){
        // What happened? We were such good friends.
        relationship[irInputBuffer[1]] = 0;
        affection[irInputBuffer[1]] = 0;
        // decline
        irSendByte(1);
      }else if(affection[irInputBuffer[1]] < enemyProposalThreshold){
        // I don't like you either.
        relationship[irInputBuffer[1]] = 2;
        // accept
        irSendByte(0);
      }else{
        // Nope. I don't think we're ready for that type of relationship.
        // decline
        irSendByte(1);
      }
    }
    irSendStopBit();
    
  }else if(irInputBuffer[2] == typeProposalReply){
    if(!waitingForReply){
      // huh?
      return;
    }
    waitingForReply = false;
    if(irInputBuffer[3] == 0){
      if(irInputBuffer[4] == 0){
        // made a friend
        relationship[irInputBuffer[1]] = 1;
      }else if(irInputBuffer[4] == 1){
        // no friend
        relationship[irInputBuffer[1]] = 0;
        affection[irInputBuffer[1]]--;
      }
    }else if(irInputBuffer[3] == 0){
      if(irInputBuffer[4] == 0){
        // made an enemy
        relationship[irInputBuffer[1]] = 2;
      }else if(irInputBuffer[4] == 1){
        // no enemy
        relationship[irInputBuffer[1]] = 0;
        affection[irInputBuffer[1]]++;
      }
    }
  }
}

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
  if((PINB&4) > 0){
    // it was noise
    irStartReceive = false;
    return 2;
  }
  
  // check the start interval length
  irTimeout = millis();
  while(((PINB&4) == 0)){ // wait for rise
    if(millis()-irTimeout > 20){
      irStartReceive = false;
      return 3;
    }
  }
  irTimeout = millis();
  while(((PINB&4) > 0)){ // wait for fall
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
  if(irInputBuffer[2] == typeAckRequest){
    bytesRemaining = 1;
  }else if(irInputBuffer[2] == typeSetCount){
    bytesRemaining = 1;
  }else if(irInputBuffer[2] == typeReset){
    bytesRemaining = 0;
  }else if(irInputBuffer[2] == typeInit){
    bytesRemaining = 6;
  }else if(irInputBuffer[2] == typeRequestPars){
    bytesRemaining = 0;
  }else if(irInputBuffer[2] == typeSendAck){
    bytesRemaining = 1;
  }else if(irInputBuffer[2] == typeSendPars){
    bytesRemaining = 6 + 7*nodeCount;
  }else if(irInputBuffer[2] == typeExchangeSelf){
    bytesRemaining = 2;
  }else if(irInputBuffer[2] == typeExchangeSelfReply){
    bytesRemaining = 2;
  }else if(irInputBuffer[2] == typeExchangeOther){
    bytesRemaining = 3;
  }else if(irInputBuffer[2] == typeExchangeOtherReply){
    bytesRemaining = 3;
  }else if(irInputBuffer[2] == typeProposal){
    bytesRemaining = 1;
  }else if(irInputBuffer[2] == typeProposalReply){
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
    while(((PINB&4) == 0)){ // wait for rise
      if(millis()-irTimeout > 20){
        return false;
      }
    }
    irTimeout = millis();
    while(((PINB&4) > 0)){ // wait for fall
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
    irInputBuffer[irBufferIndex] = b;
    irBufferIndex = irBufferIndex+1;
  }
  // else ignore it
  
  return true;
}

void irWaitForStopBit(){
  irTimeout = millis();
  while(((PINB&4) == 0)){ // wait for rise
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
  digitalWrite(txpin, LOW);
  delayMicroseconds(headerOff);
}

void irSendByte(uint8_t output){
  for(int8_t i=7; i>=0; i--){
    if(output & (1<<i)){
      txOn;
      delayMicroseconds(pulseLength);
      txOff;
      digitalWrite(txpin, LOW);
      delayMicroseconds(oneBit);
    }else{
      txOn;
      delayMicroseconds(pulseLength);
      txOff;
      digitalWrite(txpin, LOW);
      delayMicroseconds(zeroBit);
    }
  }
}

void irSendStopBit(){
  // stop bit
  txOn;
  delayMicroseconds(pulseLength);
  txOff;
  digitalWrite(txpin, LOW);
  delay(1);
}
