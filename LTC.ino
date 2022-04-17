/*
   Dynamixel : AX-series with Protocol 1.0
   Controller : OpenCM9.0
  C + OpenCM 485 EXP
   Power Source : SMPS 12V 5A

   AX-Series are connected to Dynamixel BUS on OpenCM 485 EXP board or DXL TTL connectors on OpenCM9.04
   http://emanual.robotis.com/docs/en/parts/controller/opencm485exp/#layout

   This example will test only one Dynamixel at a time.
*/

#include <DynamixelSDK.h>

// AX-series Control table address
#define ADDR_AX_TORQUE_ENABLE           64                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           104                 //Goal Position 116, Goal Velocity 104
#define ADDR_AX_PRESENT_POSITION        128

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                          1                   // Dynamixel ID: 1
#define DXL_ID2                          2
#define BAUDRATE                        57600
#define DEVICENAME                      "1"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
//DEVICENAME "2" -> Serial2
//DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
//#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
//#define DXL_MAXIMUM_POSITION_VALUE      1000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
//#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

// アナログピン
const int analogInPin0 = 0; // Analog input pin that the potentiometer
const int analogInPin1 = 1;
const int analogInPin2 = 2;
const int analogInPin3 = 3;
const int analogInPin4 = 4;

// These variables will change:
int sensorValue0 = 0;        // value read from the pot
int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;
int sensorValue4 = 0;

void setup() {
  // put your setup code here, to run once:
  // Configure the ADC pin
  pinMode(analogInPin0, INPUT_ANALOG);
  pinMode(analogInPin1, INPUT_ANALOG);
  pinMode(analogInPin2, INPUT_ANALOG);
  pinMode(analogInPin3, INPUT_ANALOG);
  pinMode(analogInPin4, INPUT_ANALOG);
  Serial.begin(115200);

  //while(!Serial);


  Serial.println("Start..");


  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  //int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int stan = 340;                               // 基準となる回転速度
  int dxl_goal_position[2] = { -stan, stan};       // モータの回転速度を格納する配列
  int sensor_state[5] = {0, 0, 0, 0, 0};          // どのセンサが反応したかを表す配列
  int history[2] = { -stan, stan};                // 1つ前のモータの値

  uint8_t dxl_error = 0;                          // Dynamixel error
  int16_t dxl_present_position = 0;               // Present position

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  /*
    if (portHandler->setBaudRate(BAUDRATE))
    {
    Serial.print("Succeeded to change the baudrate!\n");
    }
    else
    {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
    }
  */

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  else
  {
    Serial.print("Dynamixel has been successfully connected \n");
  }


  while (1)
  {
    //Serial.print("Press any key to continue! (or press q to quit!)\n");

    /*
    while(Serial.available()==0);

    int ch;

    ch = Serial.read();
    if( ch == 'q' )
      break;
    */

    // read the analog in value:
    sensorValue0 = analogRead(analogInPin0);        // センサの値を読み取る
    sensorValue1 = analogRead(analogInPin1);
    sensorValue2 = analogRead(analogInPin2);
    sensorValue3 = analogRead(analogInPin3);
    sensorValue4 = analogRead(analogInPin4);
    // map it to the range of the analog out:
    //outputValue = map(sensorValue, 0, 1023, 0, 255);
    // change the analog out value:
    //analogWrite(BOARD_LED_PIN, outputValue);

    // print the results to the serial monitor:
    Serial.print("sensor0 = " );                  // センサ値をprint
    Serial.println(sensorValue0);
    Serial.print("sensor1 = " );
    Serial.print(sensorValue1);
    Serial.print(" sensor2 = " );
    Serial.println(sensorValue2);
    Serial.print("sensor3 = " );
    Serial.print(sensorValue3);
    Serial.print(" sensor4 = " );
    Serial.println(sensorValue4);
    Serial.println();

    if (sensorValue0 >= 20) {                       // 閾値からセンサが反応したか否かを判断する
      sensor_state[0] = 1;
    } else {
      sensor_state[0] = 0;
    }
    if (sensorValue1 >= 20) {
      sensor_state[1] = 1;
    } else {
      sensor_state[1] = 0;
    }
    if (sensorValue2 >= 10) {
      sensor_state[2] = 1;
    } else {
      sensor_state[2] = 0;
    }
    if (sensorValue3 >= 20) {
      sensor_state[3] = 1;
    } else {
      sensor_state[3] = 0;
    }
    if (sensorValue4 >= 20) {
      sensor_state[4] = 1;
    } else {
      sensor_state[4] = 0;
    }

    if (sensor_state[2] == 1) {                 // ？？●？？  真ん中が反応したら、
      dxl_goal_position[0] = -stan - 10;      // 気持ち速めに前進
      dxl_goal_position[1] = stan + 10;
      //Serial.print("○○●○○");
      if (sensor_state[0] == 0 and sensor_state[1] == 1 and sensor_state[3] == 0 and sensor_state[4] == 0) {         // ○●●○○
        dxl_goal_position[0] = -stan - 20;    // やや左に曲がる
        dxl_goal_position[1] = stan - 20;
        Serial.print("○●●○○");
      } else if (sensor_state[0] == 0 and sensor_state[1] == 0 and sensor_state[3] == 1 and sensor_state[4] == 0) {  // ○○●●○
        dxl_goal_position[0] = -stan + 20;    // やや右に曲がる
        dxl_goal_position[1] = stan + 20;
        Serial.print("○○●●○");

      } else if (sensor_state[1] == 1 and sensor_state[3] == 1) {      // ○●●●○      交差
        dxl_goal_position[0] = history[0];          // 1つ前のモータの値をそのまま今回のモータの値にする
        dxl_goal_position[1] = history[1];
      }/* else if(sensor_state[0]==1 or sensor_state[4]==1 or (sensor_state[2]==1 and sensor_state[3]==1)) {
          dxl_goal_position[0] = -50;
          dxl_goal_position[1] = 50;
        } */
    } else if (sensor_state[2] == 0) {        // ？？○？？      真ん中が反応していないなら、
      if (sensor_state[0] == 1 and sensor_state[1] == 0 and sensor_state[3] == 0 and sensor_state[4] == 0) { // ●○○○○
        dxl_goal_position[0] = -stan - 80;      // めっちゃ左に曲がる
        dxl_goal_position[1] = stan - 150;
        Serial.print("●○○○○");
      } else if (sensor_state[0] == 1 and sensor_state[1] == 1 and sensor_state[3] == 0 and sensor_state[4] == 0) { // ●●○○○
        dxl_goal_position[0] = -stan - 50;      // ほんの少し強めに左に曲がる
        dxl_goal_position[1] = stan - 110;
        Serial.print("●●○○○");
      } else if (sensor_state[0] == 0 and sensor_state[1] == 1 and sensor_state[3] == 0 and sensor_state[4] == 0) { // ○●○○○
        dxl_goal_position[0] = -stan - 25;      // 左に曲がる
        dxl_goal_position[1] = stan - 65;
        Serial.print("○●○○○");

      } else if (sensor_state[0] == 0 and sensor_state[1] == 0 and sensor_state[3] == 0 and sensor_state[4] == 1) { // ○○○○●
        dxl_goal_position[0] = -stan + 150;     // めっちゃ右に曲がる
        dxl_goal_position[1] = stan + 80;
        Serial.print("○○○○●");
      } else if (sensor_state[0] == 0 and sensor_state[1] == 0 and sensor_state[3] == 1 and sensor_state[4] == 1) { // ○○○●●
        dxl_goal_position[0] = -stan + 110;     // ほんの少し強めに右に曲がる
        dxl_goal_position[1] = stan + 50;
        Serial.print("○○○●●");
      } else if (sensor_state[0] == 0 and sensor_state[1] == 0 and sensor_state[3] == 1 and sensor_state[4] == 0) { // ○○○●○
        dxl_goal_position[0] = -stan + 65;      // 右に曲がる
        dxl_goal_position[1] = stan + 25;
        Serial.print("○○○●○");
      } else if (sensor_state[0] == 0 and sensor_state[1] == 0 and sensor_state[2] == 0 and sensor_state[3] == 0 and sensor_state[4] == 0) {    // ○○○○○　全部黒で反応しなかったら、
        dxl_goal_position[0] = 50;              // めっちゃゆっくり後退
        dxl_goal_position[1] = -50;
        Serial.print("○○○○○");
      } else {                                                              // ありえないセンサ値なら、
        dxl_goal_position[0] = -110;            // ゆっくり前進
        dxl_goal_position[1] = 100;
      }
    }
    history[0] = dxl_goal_position[0];        // 今回のモータの値をhistoryに格納、次が交差ならこの値を使う
    history[1] = dxl_goal_position[1];



    // 重み付けでやろうとしたけど、上手くいきませんでした。
    /*if (back == 1) {
      delay(1000);
      back = 0;
      } else {
      if (sensorValue2 > 20 and sensorValue0 < 12 and sensorValue4 < 12) {
        dxl_goal_position[0] = -stan;
        dxl_goal_position[1] = stan;
      } else if (sensorValue0 < 12 and sensorValue1 < 12 and sensorValue2 < 7 and sensorValue3 < 12 and sensorValue4 < 12) { // 黒なら後退
        dxl_goal_position[0] = 320;
        dxl_goal_position[1] = -300;
        back = 1;
      } else {
        vector = (-(10 / 9) * sensorValue0 * 6 - (10 / 8) * sensorValue1 + (100 / 65) * sensorValue3 + (100 / 105) * sensorValue4 * 7)/2;
        dxl_goal_position[0] = -stan + ((int)vector);
        dxl_goal_position[1] = stan + ((int)vector);
        //history[1] = history[0];    // 更新
        //history[0] = (int)vector;
      }
      }
    */



    //Serial.print("\t output = ");
    //Serial.println(outputValue);
    delay(100);

    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID1, ADDR_AX_GOAL_POSITION, dxl_goal_position[0], &dxl_error);         // result、モータに伝える
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_AX_GOAL_POSITION, dxl_goal_position[1], &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }

    do
    {
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID1, ADDR_AX_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID2, ADDR_AX_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }

      /*
            Serial.print("[ID:");      Serial.print(DXL_ID);
            Serial.print(" GoalPos:"); Serial.print(dxl_goal_position[index]);
            Serial.print(" PresPos:");  Serial.print(dxl_present_position);
            Serial.println(" ");
      */

    } while (/*(abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)*/0);

    /*
      // Change goal position
      if (index == 0)
      {
      index = 1;
      }
      else
      {
      index = 0;
      }
    */
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // Close port
  portHandler->closePort();

}

void loop() {
  // put your main code here, to run repeatedly:

}

