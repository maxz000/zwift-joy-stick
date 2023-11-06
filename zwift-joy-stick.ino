#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#include <BLE2904.h>
#include <time.h>

#define BLE_DELAY 100
#define LONG_DELAY 500
#define POWER_UPDATE_DELAY 1000

#define DEBUG true

// Power definitions
// #define USE_POWER true

#define cyclingPowerService BLEUUID((uint16_t)0x1818) // uuid for the CPS from the BLE GATT website

// required characteristcs according to the CPS specification
BLECharacteristic cyclingPowerLocationCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);
BLECharacteristic cyclingPowerFeatureCharacteristic(BLEUUID((uint16_t)0x2A65), BLECharacteristic::PROPERTY_READ);
BLECharacteristic cyclingPowerMeasurementCharacteristic(BLEUUID((uint16_t)0x2A63), BLECharacteristic::PROPERTY_NOTIFY);

// Steering Definitions
#define USE_STEERING true
// #define USE_STEERING_AUTH true  // smth strange with auth, dont turn on, actually zwift doesnt request authChallenge

#define STEERING_SERVICE_UUID BLEUUID("347b0001-7635-408b-8918-8ff3949ce592")
#define STEERING_UNKNOWN_INDICATE_UUID BLEUUID("347b0014-7635-408b-8918-8ff3949ce592") // indicate
#define STEERING_ANGLE_CHAR_UUID BLEUUID("347b0030-7635-408b-8918-8ff3949ce592")       // notify
#define STEERING_RX_CHAR_UUID BLEUUID("347b0031-7635-408b-8918-8ff3949ce592")          // write
#define STEERING_TX_CHAR_UUID BLEUUID("347b0032-7635-408b-8918-8ff3949ce592")          // indicate

BLECharacteristic steeringUnknownCharacteristic(STEERING_UNKNOWN_INDICATE_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic steeringAngleCharacteristic(STEERING_ANGLE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic steeringRXCharacteristic(STEERING_RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic steeringTXCharacteristic(STEERING_TX_CHAR_UUID, BLECharacteristic::PROPERTY_INDICATE);

uint8_t FF = 0xFF;
uint8_t challengeRequest[2] = {0x03, 0x10};
uint8_t authChallenge[4] = {0x03, 0x10, 0x4a, 0x89};
uint8_t otherRequest[2] = {0x03, 0x11};
uint8_t authSuccess[4] = {0x03, 0x11, 0xff, 0xff};
uint8_t authChallengeStep = 0;
bool auth = false;

// Power stuff
uint16_t powerOut = 100; // W, decimal
uint8_t tick = 0;
auto t = millis();

// all the required definitions in binary
uint16_t cyclingPowerMeasurementCharacteristicDef = 0b0000000000100000; // cycle power config flags
bool _BLEClientConnected = false;
uint8_t cyclingPowerMeasurementCharacteristicData[8] = {(uint8_t)(cyclingPowerMeasurementCharacteristicDef & 0xff), (uint8_t)(cyclingPowerMeasurementCharacteristicDef >> 8), // flags
                                                        (uint8_t)(powerOut & 0xff), (uint8_t)(powerOut >> 8),                                                                 // inst. power
                                                        0, 0,                                                                                                                 // cum. crank
                                                        0, 0};                                                                                                                // crank time

uint32_t cyclingPowerFeatureCharacteristicDef = 0b00000000000100000000000000000000; // 000000000000000000000b;
//                                                          ^^                ^
//                                                          ||                Crank Revolution Data Supported = 1 true, 0 false
//                                                          Distributed System Support: 00=Legacy Sensor (all power, we must *2), 01=Not for distributed system (all power, we must *2), 10=Can be used in distributed system (if only one sensor connected - Collector may double the value)

uint8_t cyclingPowerFeatureCharacteristicData[4] = {(uint8_t)(cyclingPowerFeatureCharacteristicDef & 0xff), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 8), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 16), (uint8_t)(cyclingPowerFeatureCharacteristicDef >> 24)};
byte sensLoc[1] = {5};

#define STEERING_POT 33 // Joystick Xaxis to GPIO32
#define INVERT_STEERING // invert Steering axis
#define POWER_POT 32    // Joystick Yaxis to GPIO33
#define INVERT_POWER    // invert Power axis
#define GND_POT 19
#define V_POT 18

float angle = 0;
float angle_deviation = 0;
float power_deviation = 0;

// Angle calculation parametres
#define MAX_ADC_RESOLUTION 4095 // ESP32 ADC is 12bit
#define HALF_ADC_RESOLUTION 2047
#define MAX_STEER_ANGLE 35
#define MAX_POWER_VALUE 300
#define ZERO_FLOOR 3

class MyServerCallbacks : public BLEServerCallbacks
{ // class for disconnect connect events catching
  void onConnect(BLEServer *pServer)
  {
    _BLEClientConnected = true;
    auth = false;
    authChallengeStep = 0;
#ifdef DEBUG
    Serial.println("BLE Client connected");
#endif
  };

  void onDisconnect(BLEServer *pServer)
  {
    _BLEClientConnected = false;
    auth = false;
    authChallengeStep = 0;
#ifdef DEBUG
    Serial.println("BLE Client disconnected");
#endif
    BLEDevice::startAdvertising();
  }
};

class SteeringRXCharacteristicCallbacks : public BLECharacteristicCallbacks
{

  void onRead(BLECharacteristic *pRx)
  {
  }

  void onWrite(BLECharacteristic *pRx)
  {
    std::string rxValue = pRx->getValue();
#ifdef DEBUG
    Serial.println("RX new value");
    for (int i = 0; i < 4; i++)
    {
      Serial.print(rxValue[i], HEX);
      Serial.print('-');
    }
    Serial.println();
    Serial.print("auth: ");
    Serial.print(auth);
    Serial.print(" challenge step ");
    Serial.println(authChallengeStep);
#endif
#ifdef USE_STEERING_AUTH
    if (!auth)
    {
      if (rxValue[0] == challengeRequest[0] && rxValue[1] == challengeRequest[1])
      {
        steeringTXCharacteristic.setValue(authChallenge, 4);
        steeringTXCharacteristic.indicate();
        authChallengeStep = 1;
      }
      if (rxValue[0] == otherRequest[0] && rxValue[1] == otherRequest[1])
      {
        steeringTXCharacteristic.setValue(authSuccess, 4);
        steeringTXCharacteristic.indicate();
        auth = true;
        authChallengeStep = 2;
      }
    }
#endif
#ifdef DEBUG
    if (auth)
    {
      Serial.println("Auth success");
    }
    else
    {
      Serial.println("Auth not success");
    }
#endif
  }
};

void InitBLE()
{ // BLE initialization proc
#ifdef DEBUG
  Serial.println("INIT BLE");
#endif
  BLEDevice::init("POWER_CHEAT");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  pServer->setCallbacks(new MyServerCallbacks());

#ifdef USE_POWER
  // Create the BLE power Service
  BLEService *pPower = pServer->createService(cyclingPowerService);

  pPower->addCharacteristic(&cyclingPowerMeasurementCharacteristic);
  cyclingPowerMeasurementCharacteristic.addDescriptor(new BLE2902());

  pPower->addCharacteristic(&cyclingPowerFeatureCharacteristic);
  pPower->addCharacteristic(&cyclingPowerLocationCharacteristic);

  advertising->addServiceUUID(cyclingPowerService);

  pPower->start();
#ifdef DEBUG
  Serial.println("Power service started");
#endif
#endif

#ifdef USE_STEERING
  // Create the BLE steering Service
  BLEService *pSteering = pServer->createService(STEERING_SERVICE_UUID);

  pSteering->addCharacteristic(&steeringUnknownCharacteristic);
  steeringUnknownCharacteristic.addDescriptor(new BLE2902());

  pSteering->addCharacteristic(&steeringAngleCharacteristic);
  steeringAngleCharacteristic.addDescriptor(new BLE2902());

  pSteering->addCharacteristic(&steeringTXCharacteristic);

  pSteering->addCharacteristic(&steeringRXCharacteristic);
  steeringRXCharacteristic.addDescriptor(new BLE2902());

  uint8_t defaultValue[4] = {0x0, 0x0, 0x0, 0x0};
  steeringAngleCharacteristic.setValue(defaultValue, 4);
  defaultValue[0] == FF;
  steeringUnknownCharacteristic.setValue(defaultValue, 1);
  steeringRXCharacteristic.setValue(defaultValue, 1);
  steeringTXCharacteristic.setValue(authChallenge, 4);

  steeringTXCharacteristic.setCallbacks(new SteeringRXCharacteristicCallbacks());
  steeringRXCharacteristic.setCallbacks(new SteeringRXCharacteristicCallbacks());

  advertising->addServiceUUID(STEERING_SERVICE_UUID);

  pSteering->start();

#if DEBUG
  Serial.println("Steering service started");
#endif
#endif

  // Start advertising
#ifdef DEBUG
  Serial.println("Start advertising");
#endif

  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

#ifdef DEBUG
  Serial.println("INIT BLE Done");
#endif
}

float readAngle()
{
  int potVal = analogRead(STEERING_POT);
  /* Old Style calc.
  angle = map(potVal,0,4095,-35,35); //Mapping function
  */

  // kwakeham style:
  angle = (((potVal) / (float)MAX_ADC_RESOLUTION) * (MAX_STEER_ANGLE * 2)) - MAX_STEER_ANGLE;

  if (fabsf(angle) < ZERO_FLOOR)
  {
    angle = 0;
  }
#ifdef INVERT_STEERING
  angle = angle * (-1);
#endif

  return angle - angle_deviation;
}

uint16_t readPower()
{
  uint16_t potVal = analogRead(POWER_POT);
#ifdef INVERT_POWER
  if (potVal > HALF_ADC_RESOLUTION)
  {
    potVal = HALF_ADC_RESOLUTION;
  }
  potVal = HALF_ADC_RESOLUTION - potVal;
#else
  if (potVal < HALF_ADC_RESOLUTION)
  {
    potVal = HALF_ADC_RESOLUTION;
  }
  potVal -= HALF_ADC_RESOLUTION;
#endif
  return powerOut + (uint16_t)(((potVal) / (float)HALF_ADC_RESOLUTION) * MAX_POWER_VALUE);
}

void InitJoystick()
{
  pinMode(GND_POT, OUTPUT);
  pinMode(V_POT, OUTPUT);
  pinMode(STEERING_POT, INPUT);
  pinMode(POWER_POT, INPUT);
  digitalWrite(V_POT, HIGH);
  digitalWrite(GND_POT, LOW);
  angle_deviation = readAngle();
  // power_deviation = analogRead(POWER_POT);
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Start");
#endif

  InitJoystick();
  delay(LONG_DELAY);

  InitBLE();
  delay(LONG_DELAY);
  t = millis();
}

void loop()
{
  t = millis();
  tick += 1;
#ifdef DEBUG
  if (tick % 40 == 0)
  {
    Serial.print("Tick ");
    Serial.print(tick);
    Serial.print(" time ");
    Serial.println(t);
  }
#endif
  uint16_t powerOutValue = readPower();
  float angle = readAngle();

  if (true)
  {
#if DEBUG
    if (tick % 20 == 0)
    {
      Serial.print("notify ");
#ifdef USE_STEERING
      Serial.print("Angle: ");
      Serial.print(angle);
#endif
#ifdef USE_POWER
      Serial.print(" Power: ");
      Serial.print(powerOutValue);
#endif
      Serial.println(".");
    }
#endif
#ifdef USE_POWER
    if (tick % 2 == 0)
    { // send power every second tick ~ 1rps
      // prepare sending data array
      cyclingPowerMeasurementCharacteristicData[2] = (uint8_t)(powerOutValue & 0xff);
      cyclingPowerMeasurementCharacteristicData[3] = (uint8_t)(powerOutValue >> 8);

      cyclingPowerMeasurementCharacteristic.setValue(cyclingPowerMeasurementCharacteristicData, 8); // prepare sending data array
      cyclingPowerMeasurementCharacteristic.notify();                                               // send data of curent measurement
                                                                                                    // cyclingPowerFeatureCharacteristic.setValue(cyclingPowerFeatureCharacteristicData, 4);         // update data of powermeter features
                                                                                                    // cyclingPowerLocationCharacteristic.setValue(sensLoc, 1);                                      // update data powermeter location = left crunck
    }
#endif

#if defined(USE_STEERING) && defined(USE_STEERING_AUTH)
    if (auth)
    {
      steeringAngleCharacteristic.setValue(angle);
      steeringAngleCharacteristic.notify();
    }
#elif defined(USE_STEERING)
    steeringAngleCharacteristic.setValue(angle);
    steeringAngleCharacteristic.notify();
#endif
  }
  delay(constrain(POWER_UPDATE_DELAY / 2 - (millis() - t), 0, POWER_UPDATE_DELAY / 2)); // make 1 loop tick ~ 500 millis
}
