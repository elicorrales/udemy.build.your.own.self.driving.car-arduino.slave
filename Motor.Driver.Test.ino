
#define ENABLE_LEFT_MOTOR  5             //pin ENA on L298N motor driver board
#define LEFT_MOTOR_A        6             //pin IN1     "
#define LEFT_MOTOR_B        7             //pin IN2     "

#define ENABLE_RIGHT_MOTOR 10            //pin ENB     "
#define RIGHT_MOTOR_A       8             //pin IN3     "
#define RIGHT_MOTOR_B       9             //pin IN4     "

#define  ENDMARKER '\n'


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GLOBAL DEFINES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MYDEBUG 0


#define MAX_RX_PARM_BUF 11
#define MAX_TX_BUF 64

#define DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST 2000  //30
#define TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME 100

#define ENABLE_LEFT_MOTOR  5             //pin ENA on L298N motor driver board
#define LEFT_MOTOR_A        6             //pin IN1     "
#define LEFT_MOTOR_B        7             //pin IN2     "

#define ENABLE_RIGHT_MOTOR 10            //pin ENB     "
#define RIGHT_MOTOR_A       8             //pin IN3     "
#define RIGHT_MOTOR_B       9             //pin IN4     "

#define  ENDMARKER '\n'

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE COMMANDS THE ARDUINO UNDERSTANDS RECEIVED VIA USB SERIAL
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum COMMAND {
  CLRAllERRORS = 4,
  RSTNUMUSBCMDS = 5,
  MOVETIMEOUT = 6,
  STATUS = 24,
  STOP = 28,
  FORWARD = 29,
  BACKWARD = 32,
  LEFT = 33,
  RIGHT = 34,
};

enum DIRECTION {
  DIR_STOPPED = 0,
  DIR_FORWARD = 1,
  DIR_BACKWARD = 2,
  DIR_LEFT = 3,
  DIR_RIGHT = 4,
};

const char* bJSON = "{";
const char* eJSON = "}";
const char* sep = ",";
const char* v = "\"v\":";                 //volts
const char* a1 = "\"a1\":";               //amps1
const char* a2 = "\"a2\":";               //amps2
const char* t = "\"t\":";                 //temp
const char* s1 = "\"s1\":";               //speed M1
const char* s2 = "\"s2\":";               //speed M2
const char* c = "\"c\":";                 //num commands received
const char* d = "\"d\":";                 //num rxd dropped usb strings
const char* l = "\"l\":";                 //the last command received
const char* p1str= "\"p1\":";                //the last param1 received
const char* e = "\"e\":";                 //error
const char* m = "\"m\":";                 //message
const char* quote = "\"";
const char* negOne  = "-1";
const char* isEmptyStr = " is empty\"";
const char* badChksum  = "\"Bad Chksum [";
const char* bRxdCmd    = "\"Rxd Cmd [";
const char* badNParms  = "\"Bad nParm [";
const char* bUnkCmd    = "\"UNKCMD [";
const char* bracketEOS = "]\"";

const char* dir = "\"dir\":";
const char* forward = "\"forward\",";
const char* backward = "\"backward\",";
const char* left = "\"left\",";
const char* right = "\"right\",";
const char* busy = "\"busy\",";
const char* drvAck = "\"ackNum\":";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE SERIAL COMM BUFERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////// this is the main serial input buffer to temporarily store whatever came in from USB serial /////////
char mainTxRxBuffer[SERIAL_RX_BUFFER_SIZE+1] = {'\0'};   // an array to store the received data
char tempHoldingBuf[MAX_RX_PARM_BUF] = {'\0'};

/////////////// these small buffers store whatever was parsed from the above input USB serial buffer ////////////
char numParms[MAX_RX_PARM_BUF] = {'\0'};
char command[MAX_RX_PARM_BUF] = {'\0'};
char randNum[MAX_RX_PARM_BUF] = {'\0'};
char chksum[MAX_RX_PARM_BUF] = {'\0'};
char param1[MAX_RX_PARM_BUF]  = {'\0'};
char param2[MAX_RX_PARM_BUF]  = {'\0'};



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE PROGRAM FLOW-CONTROL FLAGS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool userHasInitiatedArduino = false;
bool thereIsUsbError = true;  //initialized to true, to force client program (Raspberry?  Browser?) to clear the flag
bool newData = false;
bool newCommandIsReady = false;
bool motorsAreStopping = false;

/////////// stuff related to auto-sending status back to host /////////////////////////////////////

bool continueToAutoSendStatusToHost = true;
unsigned long prevMillisLastTimeAutoSentStatus = millis();
unsigned long autoSendStatusTimeout = DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST;
unsigned long giveMotorsTimeToStopMillis = millis();

//////////// these are roboclaw values that are set as a result of calling roboclaw library functions.
bool rcValid;
uint8_t rcStatus;

//////////// these are the other global values related to the roboclaw or arduino, of interest
int nParms = -1;
int cmd = -1;
int rnd = -1;
int chksm = -1;
int p1  = -1;
int p2  = -2;
float p1f = -1.0;
float p2f = -1.0;
int movementTimeoutMs = TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME;
char version[32];
char lastError[MAX_TX_BUF] = {'\0'};
char msg[MAX_TX_BUF] = {'\0'};
long numCmdsRxdFromUSB = 0;
int lastCmdRxdFromUSB = -1;
long numDroppedUsbPackets = 0;
byte currSpeedCommanded = 0;
unsigned long prevMillisLastCommand = millis();

////////////  we need to know if motors were commanded to turn, as a safety feature, so we can shut them down.
bool motorM1Rotating = false;
bool motorM2Rotating = false;

DIRECTION prevDirection = DIR_STOPPED;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Typical Arduino setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(115200); //9600 19200 38400 57600 74880 115200 230400 250000 500000 1000000 2000000
  while (!Serial);

  pinMode(ENABLE_LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);

  pinMode(ENABLE_RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  
  Serial.println(F("{\"msg\":\"Arduino Motor Driver Is Up\"}"));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // safety and / or status related functions
  stopMotorsIfBeenTooLongSinceLastCommand();
  respondWithStatusIfBeenTooLongSinceLastTime();

  recvIncomingUsbSerialWithEndMarker();
  parseIncomingUsbSerial();
  commandHandler();

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start of Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void commandHandler() {

  if (!newCommandIsReady) return;

  if (cmd != STATUS) {
    lastCmdRxdFromUSB = -1; lastCmdRxdFromUSB = cmd;
    numCmdsRxdFromUSB++;
  }


  switch (cmd) {

    //////////////////// Arduino user - instructions /////////////////////////
    case CLRAllERRORS:
      userHasInitiatedArduino = true;
      thereIsUsbError = false;
      break;
    case RSTNUMUSBCMDS:
      numCmdsRxdFromUSB = 0;
      numDroppedUsbPackets = 0;
      break;
    case MOVETIMEOUT:
      setMovementTimeoutMs();

    //////////////////// Roboclaw instructions /////////////////////////
    case STATUS: //read volts, amps, temp
      readStatus();
      break;
    case STOP: //stop both motors
      stopMotors();
      break;
    case FORWARD:
      prevMillisLastCommand = millis();
      if (!userHasInitiatedArduino || thereIsUsbError) {
        break;
      }
      moveForward();
      break;
    case BACKWARD:
      prevMillisLastCommand = millis();
      if (!userHasInitiatedArduino || thereIsUsbError) {
        break;
      }
      moveBackward();
      break;
    case LEFT:
      prevMillisLastCommand = millis();
      if (!userHasInitiatedArduino || thereIsUsbError) {
        break;
      }
      rotateLeft();
      break;
    case RIGHT:
      prevMillisLastCommand = millis();
      if (!userHasInitiatedArduino || thereIsUsbError) {
        break;
      }
      rotateRight();
      break;

    default:
      memset(lastError, 0x00, MAX_TX_BUF);
      strncpy(lastError, bUnkCmd, strlen(bUnkCmd));
      strncat(lastError, command, strlen(command));
      strncat(lastError, bracketEOS, strlen(bracketEOS));
      thereIsUsbError = true;
      stopMotors();
      readStatus();
  }

  //this may be optional.. but the Roboclaw takes a certain amount of time to do stuff
  //and I didnt want to be stepping on it by having yet another command (or other function)
  // interfering during that time.
  delay(10);

  newCommandIsReady = false;
  newData = false;


}

//////////////////// Arduino user - instructions functions /////////////////////////
//////////////////// Arduino user - instructions functions /////////////////////////
//////////////////// Arduino user - instructions functions /////////////////////////

void setMovementTimeoutMs() {
  movementTimeoutMs = p1;
}

void stopAutoSendingStatusToHost() {
  continueToAutoSendStatusToHost = false;
}

void startAutoSendingStatusToHost() {
  continueToAutoSendStatusToHost = true;
  autoSendStatusTimeout = strtol(param1, NULL, 10);
}




void sendDriveAck(char* directionStr, int ackNum) {
  memset(mainTxRxBuffer, 0x00, SERIAL_RX_BUFFER_SIZE);
  strncpy(mainTxRxBuffer, bJSON, strlen(bJSON));
  strncat(mainTxRxBuffer, dir, strlen(dir));
  strncat(mainTxRxBuffer, directionStr, strlen(directionStr));
  strncat(mainTxRxBuffer, drvAck, strlen(drvAck));
  memset(tempHoldingBuf, 0x00, MAX_RX_PARM_BUF);
  itoa(ackNum, tempHoldingBuf, 10);
  strncat(mainTxRxBuffer, tempHoldingBuf, strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer, eJSON, strlen(eJSON));
  Serial.println(mainTxRxBuffer);
}
void readStatus() {

  memset(mainTxRxBuffer, 0x00, SERIAL_RX_BUFFER_SIZE);

  strncpy(mainTxRxBuffer, bJSON, strlen(bJSON));

  strncat(mainTxRxBuffer, c, strlen(c));
  memset(tempHoldingBuf, 0x00, MAX_RX_PARM_BUF);
  itoa(numCmdsRxdFromUSB, tempHoldingBuf, 10);
  strncat(mainTxRxBuffer, tempHoldingBuf, strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer, sep, strlen(sep));

  strncat(mainTxRxBuffer, d, strlen(d));
  memset(tempHoldingBuf, 0x00, MAX_RX_PARM_BUF);
  itoa(numDroppedUsbPackets, tempHoldingBuf, 10);
  strncat(mainTxRxBuffer, tempHoldingBuf, strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer, sep, strlen(sep));

  strncat(mainTxRxBuffer, l, strlen(l));
  memset(tempHoldingBuf, 0x00, MAX_RX_PARM_BUF);
  if (strlen(command) > 0) {
    strncat(mainTxRxBuffer, command, strlen(command));
  } else {
    strncat(mainTxRxBuffer, negOne, strlen(negOne));
  }
  strncat(mainTxRxBuffer, sep, strlen(sep));

  strncat(mainTxRxBuffer, p1str, strlen(p1str));
  if (strlen(param1) > 0) {
    strncat(mainTxRxBuffer, param1, strlen(param1));
  } else {
    strncat(mainTxRxBuffer, negOne, strlen(negOne));
  }

  int lenOfError = strlen(lastError);
  int lenOfMsgBuf = strlen(msg);

  if (lenOfError > 0) {
    strncat(mainTxRxBuffer, sep, strlen(sep));
    strncat(mainTxRxBuffer, e, strlen(e));
    strncat(mainTxRxBuffer, lastError, strlen(lastError));
  }

  if (lenOfMsgBuf > 0) {
    strncat(mainTxRxBuffer, sep, strlen(sep));
    strncat(mainTxRxBuffer, m, strlen(m));
    strncat(mainTxRxBuffer, msg, strlen(msg));
  }

  strncat(mainTxRxBuffer, eJSON, strlen(eJSON));

  Serial.println(mainTxRxBuffer);

}

bool stoppingMotorsFirstIfMovingAndDirectionChanged(DIRECTION newDirection) {
  
  if (motorsAreStopping) {
    if (millis() - giveMotorsTimeToStopMillis > 100) {
      motorsAreStopping = false;
      giveMotorsTimeToStopMillis = millis();
      return false;
    }
    return true;
  }
  if (motorM1Rotating || motorM2Rotating) {
    if (prevDirection != DIR_STOPPED && prevDirection != newDirection) {
      motorsAreStopping = true;
      stopMotors();
      return true;
    }
  }

  return false;
}

void moveForward() {
  if (stoppingMotorsFirstIfMovingAndDirectionChanged(DIR_FORWARD)) {
    sendDriveAck(busy, p2);
    return;
  }
  prevDirection = DIR_FORWARD;
  rotateLeftSideForward(p1);
  rotateRightSideForward(p1);
  sendDriveAck(forward, p2);
}

void moveBackward() {
  if (stoppingMotorsFirstIfMovingAndDirectionChanged(DIR_BACKWARD)) {
    sendDriveAck(busy, p2);
    return;
  }
  prevDirection = DIR_BACKWARD;
  rotateLeftSideBackward(p1);
  rotateRightSideBackward(p1);
  sendDriveAck(backward, p2);
}

void rotateLeft() {
  if (stoppingMotorsFirstIfMovingAndDirectionChanged(DIR_LEFT)) {
    sendDriveAck(busy, p2);
    return;
  }
  prevDirection = DIR_LEFT;
  rotateLeftSideBackward(p1);
  rotateRightSideForward(p1);
  sendDriveAck(left, p2);
}

void rotateRight() {
  if (stoppingMotorsFirstIfMovingAndDirectionChanged(DIR_RIGHT)) {
    sendDriveAck(busy, p2);
    return;
  }
  prevDirection = DIR_RIGHT;
  rotateLeftSideForward(p1);
  rotateRightSideBackward(p1);
  sendDriveAck(right, p2);
}

void rotateLeftSideForward(int speed) {
  //Serial.println("Left Side Forward");
  digitalWrite(LEFT_MOTOR_A, HIGH);
  digitalWrite(LEFT_MOTOR_B, LOW);
  analogWrite(ENABLE_LEFT_MOTOR, map(speed, 0, 100, 0, 255));
  motorM1Rotating = true;
}

void rotateRightSideForward(int speed) {
  //Serial.println("Right Side Forward");
  digitalWrite(RIGHT_MOTOR_A, HIGH);
  digitalWrite(RIGHT_MOTOR_B, LOW);
  analogWrite(ENABLE_RIGHT_MOTOR, map(speed, 0, 100, 0, 255));
  motorM2Rotating = true;
}

void stopMotors() {
  //Serial.println("Stop Motors");
  digitalWrite(ENABLE_LEFT_MOTOR, LOW);
  digitalWrite(LEFT_MOTOR_A, LOW);
  digitalWrite(LEFT_MOTOR_B, LOW);
  digitalWrite(ENABLE_RIGHT_MOTOR, LOW);
  digitalWrite(RIGHT_MOTOR_A, LOW);
  digitalWrite(RIGHT_MOTOR_B, LOW);
  giveMotorsTimeToStopMillis = millis();
  motorM1Rotating = false;
  motorM2Rotating = false;
}

void rotateLeftSideBackward(int speed) {
  //Serial.println("Left Side Backward");
  digitalWrite(LEFT_MOTOR_A, LOW);
  digitalWrite(LEFT_MOTOR_B, HIGH);
  analogWrite(ENABLE_LEFT_MOTOR, map(speed, 0, 100, 0, 255));
  motorM1Rotating = true;
}

void rotateRightSideBackward(int speed) {
  //Serial.println("Right Side Backward");
  digitalWrite(RIGHT_MOTOR_A, LOW);
  digitalWrite(RIGHT_MOTOR_B, HIGH);
  analogWrite(ENABLE_RIGHT_MOTOR, map(speed, 0, 100, 0, 255));
  motorM2Rotating = true;
}


//////////////////// private functions /////////////////////////
//////////////////// private functions /////////////////////////
//////////////////// private functions /////////////////////////

void recvIncomingUsbSerialWithEndMarker() {
  static byte ndx = 0;
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != ENDMARKER) {
      mainTxRxBuffer[ndx] = rc;
      ndx++;
      if (ndx >= SERIAL_RX_BUFFER_SIZE) {
        ndx = SERIAL_RX_BUFFER_SIZE - 1;
      }
    }
    else {
      mainTxRxBuffer[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }

}


void parseIncomingUsbSerial() {

  if (newData != true) return;

  memset(numParms, 0x00, MAX_RX_PARM_BUF);
  memset(command, 0x00, MAX_RX_PARM_BUF);
  memset(randNum, 0x00, MAX_RX_PARM_BUF);
  memset(chksum, 0x00, MAX_RX_PARM_BUF);
  memset(param1, 0x00, MAX_RX_PARM_BUF);
  memset(param2, 0x00, MAX_RX_PARM_BUF);

  //Serial.println("..parsing..");


  byte ndx = 0;
  byte pidx = 0;
  byte sidx = 0;

  /////   extract numParms out of the receive buffer
  pidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    numParms[ndx] = mainTxRxBuffer[ndx];
    ndx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract  command out of the receive buffer
  pidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && ndx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    command[pidx] = mainTxRxBuffer[ndx];
    pidx++;
    ndx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract randNum out of the receive buffer
  pidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    randNum[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract chksum out of the receive buffer
  pidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    chksum[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }



  /////   extract param1 out of the receive buffer
  pidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    param1[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }



  /////   extract param2 out of the receive buffer
  pidx = 0;
  while (ndx < SERIAL_RX_BUFFER_SIZE && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    param2[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  memset(mainTxRxBuffer, 0x00, SERIAL_RX_BUFFER_SIZE+1);

  if (verifyChecksum()) {
    newCommandIsReady = true;
    //Serial.println(F("command verified"));
    thereIsUsbError = false;
    memset(lastError, 0x00, MAX_TX_BUF);


  } else {
    numDroppedUsbPackets++;
    //Serial.println(F("command NOT verified"));
    newData = false; // whatever was rxd from USB was bad, so we're just ignoring it and are now ready to receive more from USB
  }


}

bool verifyRequiredIncomingStringToken(char* bufName, char* buf) {
  if (strlen(buf) < 1) {

    memset(msg, 0x00, MAX_TX_BUF);
    memset(lastError, 0x00, MAX_TX_BUF);
    strncpy(lastError, quote, strlen(quote));
    strncat(lastError, bufName, strlen(bufName));
    strncat(lastError, isEmptyStr, strlen(isEmptyStr));
    readStatus();
    return false;
  }
  return true;
}

bool verifyChecksum() {

  int numberOfUsbParameters = 0;

  if (!verifyRequiredIncomingStringToken("numParms", numParms)) {
    return false;
  }

  numberOfUsbParameters++;

  if (!verifyRequiredIncomingStringToken("command", command)) {
    return false;
  }

  numberOfUsbParameters++;

  if (!verifyRequiredIncomingStringToken("randNum", randNum)) {
    return false;
  }

  numberOfUsbParameters++;

  if (!verifyRequiredIncomingStringToken("chksum", chksum)) {
    return false;
  }

  numberOfUsbParameters++;

  if (strlen(param1) > 0) numberOfUsbParameters++;
  if (strlen(param2) > 0) numberOfUsbParameters++;

  nParms = atoi(numParms);

  if (numberOfUsbParameters != nParms) {
    memset(lastError, 0x00, MAX_TX_BUF);
    memset(msg, 0x00, MAX_TX_BUF);
    strncpy(lastError, badNParms, strlen(badNParms));
    strncat(lastError, numParms, strlen(numParms));
    strncat(lastError, bracketEOS, strlen(bracketEOS));
    readStatus();
    return false;
  }

  cmd = atoi(command);
  rnd = atoi(randNum);
  chksm = atoi(chksum);
  p1 = 0;
  p1f = 0;
  p2 = 0;
  p2f = 0;
  if (isDecimalValue(param1)) {
    p1f = atof(param1);
  } else {
    p1 = atoi(param1);
  }
  if (isDecimalValue(param2)) {
    p2f = atof(param2);
  } else {
    p2 = atoi(param2);
  }


  long int mySum = nParms + cmd + rnd + (int)(p1f != 0 ? p1f * 1000 : p1) + (int)(p2f != 0 ? p2f * 1000 : p2);

  //  Serial.print(" nParms:");Serial.print(nParms);
  //  Serial.print(" cmd:");Serial.print(cmd);
  //  Serial.print(" rnd:");Serial.print(rnd);
  //  Serial.print(" p1:");Serial.print((int)(p1f!=0?p1f*1000:p1));
  //  Serial.print(" p2:");Serial.print((int)(p2f!=0?p2f*1000:p2));
  //  Serial.print(" mySum:");Serial.print(mySum);
  //  Serial.print(" chksm:");Serial.println(chksm);


  if (mySum == chksm) {
    memset(msg, 0x00, MAX_TX_BUF);
    memset(lastError, 0x00, MAX_TX_BUF);
    strncpy(msg, bRxdCmd, strlen(bRxdCmd));
    strncat(msg, command, strlen(command));
    strncat(msg, bracketEOS, strlen(bracketEOS));
    readStatus();
    return true;
  } else {
    memset(msg, 0x00, MAX_TX_BUF);
    memset(lastError, 0x00, MAX_TX_BUF);
    strncpy(lastError, badChksum, strlen(badChksum));
    strncat(lastError, chksum, strlen(chksum));
    strncat(lastError, bracketEOS, strlen(bracketEOS));
    readStatus();
    return false;
  }
}

bool isDecimalValue(char* paramBuffer) {
  byte len = strlen(paramBuffer);
  for (byte i = 0; i < len; i++) {
    if (paramBuffer[i] == '.') {
      return true;
    }
  }
  return false;
}


void stopMotorsIfBeenTooLongSinceLastCommand() {
  if (motorM1Rotating || motorM2Rotating) {
    if (millis() - prevMillisLastCommand > movementTimeoutMs) {
      stopMotors();
    }
  }
}

void respondWithStatusIfBeenTooLongSinceLastTime() {
  if (continueToAutoSendStatusToHost && ((millis() - prevMillisLastTimeAutoSentStatus) > autoSendStatusTimeout)) {
    prevMillisLastTimeAutoSentStatus = millis();
    readStatus();
  }
}
