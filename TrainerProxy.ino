/*
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
  Ported to Arduino ESP32 by Evandro Copercini

  using https://github.com/h2zero/NimBLE-Arduino/blob/master/examples/NimBLE_Client/NimBLE_Client.ino
*/

/* ESP32-WROOM-DA Module */

#include <Arduino.h>
#include <NimBLEDevice.h>

// BLE services
#define ESP_GATT_UUID_FITNESS_MACHINE_SVC 0x1826

// BLE characteristics
#define ESP_GATT_UUID_FITNESS_MACHINE_FEATURE_CHAR          0x2acc
#define ESP_GATT_UUID_CROSS_TRAINER_DATA_CHAR               0x2ace
#define ESP_GATT_UUID_INDOOR_BIKE_DATA_CHAR                 0x2ad2
#define ESP_GATT_UUID_SUPPORTED_RESISTANCE_LEVEL_RANGE_CHAR 0x2ad6
#define ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR    0x2ad9
#define ESP_GATT_UUID_FITNESS_MACHINE_STATUS_CHAR           0x2ada

// fitness machine control point: opcodes (FTMS v1.0 p.49)
#define FMCP_OPCODE_REQUEST_CONTROL           0x00  // no params
#define FMCP_OPCODE_RESET                     0x01  // no params
#define FMCP_OPCODE_SET_TARGET_SPEED          0x02  // uint16, 0.01 km/h
#define FMCP_OPCODE_SET_TARGET_INCLINATION    0x03  // int16, 0.1%
#define FMCP_OPCODE_SET_TARGET_RESISTANCE     0x04  // uint8, 0.1
#define FMCP_OPCODE_SET_TARGET_POWER          0x05  // int16, 1 W
#define FMCP_OPCODE_SET_TARGET_HEARTRATE      0x06  // uint8, 1 bpm
#define FMCP_OPCODE_START_RESUME              0x07  // no params
#define FMCP_OPCODE_STOP_PAUSE                0x08  // uint8, 1..stop 2..pause
#define FMCP_OPCODE_SET_TARGET_ENERGY         0x09  // uint16, 1 cal
#define FMCP_OPCODE_SET_TARGET_STEPS          0x0a  // uint16, 1 step
#define FMCP_OPCODE_SET_TARGET_STRIDES        0x0b  // uint16, 1 stride
#define FMCP_OPCODE_SET_TARGET_DISTANCE       0x0c  // uint24, 1 m
#define FMCP_OPCODE_SET_TARGET_TRAINING_TIME  0x0d  // uint16, 1s 
#define FMCP_OPCODE_SET_TARGET_TIME_2HR_ZONES 0x0e  // 2 uint16, 1 s fat burn zone, 1 s fitness zone
#define FMCP_OPCODE_SET_TARGET_TIME_3HR_ZONES 0x0f  // 3 uint16, 1 s light zone, 1 s moderate zone, 1 s hard zone
#define FMCP_OPCODE_SET_TARGET_TIME_5HR_ZONES 0x10  // 5 uint16, 1 s very light zone, 1 s light zone, 1s moderate zone, 1 s hard zone, 1 s maximum zone
#define FMCP_OPCODE_SET_INDOOR_BIKE_SIM       0x11  // 2 int16, 2 uint8, wind speed 0.001 m/s, grade 0.01 %, crr 0.0001, cw 0.01
#define FMCP_OPCODE_SET_WHEEL_CIRCUMFERENCE   0x12  // uint16, 0.1 mm
#define FMCP_OPCODE_SPIN_DOWN_CONTROL         0x13  // uint8, 1..start 2..ignore
#define FMCP_OPCODE_SET_TARGET_CADENCE        0x14  // uint16, 0.5 steps/min


struct devInfo
{
  float RLR_low;   // resistance level range - low limit
  float RLR_high;  // resistance level range - high limit
  float RLR_step;  // resistance level range - step size
};

struct devData
{
  uint16_t instSpeed;   // instantaneous speed [0.01 km/h]
  int16_t instPower;    // instantaneous power [W]
  uint16_t instCadence; // instantaneous cadence [0.5 1/min]
  uint8_t heartRate;    // heart rate [bpm]
};

struct ctrlData
{
  uint16_t requestedPower;       // power level requested from client (mobile app)
  uint8_t commandedResistance;  // commanded resistance (to server on fitness machine)
};

enum fsm_state
{
  FSM_UNDEFINED,
  FSM_CTRL_REQUEST,
  FSM_CTRL_REQUESTED,
  FSM_RESET_REQUEST,
  FSM_RESET_DONE,
  FSM_STOP_REQUEST,
  FSM_STOP_DONE,
  FSM_START_REQUEST,
  FSM_START_DONE,
  FSM_SET_TARGET_REQUEST,
  FSM_SET_TARGET_DONE
};

struct fsm
{
  uint32_t requests;    // bit coded requests from client (e.g. mobile app) - each bit corresponds to the FMCP opcode (i.e. opcode 0x00 is reflected in bit 0 - as requested by the client)
                        // onDisconnect from client: set bit 31, return to FSM_UNDEFINED
  enum fsm_state state; // fsm state of server (i.e. fitness machine)
};



// proxy
struct fsm fsm;
struct ctrlData ctrlData;

// client
struct devInfo devInfo;
struct devData devData;
bool foundDevice=false;
bool doConnectToServer=false;
static const NimBLEAdvertisedDevice *device;
static uint32_t scanTimeMs = 5000; /** scan time in milliseconds, 0 = scan forever */
NimBLEClient* pClient = nullptr;
uint8_t fitnessMachineControlPointNotificationValue[] = {0xff, 0x00, 0x00};

// server
static NimBLEServer *pServer;
NimBLECharacteristic *pCharIndoorBikeData;
bool doNotifyIndoorBikeData = false;              // on subscribing, notify regularly
uint16_t indoorBikeData[] = {0x0044, 0x0000, 0x0000, 0x0000}; // speed, rpm, power
NimBLECharacteristic *pCharFitnessMachineStatus;
uint8_t fitnessMachineStatus[] = {0xff};
bool doNotifyFitnessMachineStatus = false;        // on subscribing, notify back to client once changes are confirmed from server
NimBLECharacteristic *pCharFitnessMachineControlPoint;
uint8_t fitnessMachineControlPoint[] = {0x80, 0x00, 0x00, 0x00};  // response code, request opcode, result, optional parameter
bool doIndicateFitnessMachineControlPoint = false;  // on subcribing, indicate(!) success of operation back to client (in our case here, this is when the connected fitness machine executed the command and replied itself)

bool connectToServer();

class serverCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo)
  {
    Serial.printf("connect from: conn_id %s\n", connInfo.getAddress().toString().c_str());

    if (foundDevice)
    {
      Serial.printf("peer device already scanned, we can connect\n");
      doConnectToServer=true;
    }
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason)
  {
    Serial.printf("disconnect from: conn_id %s\n", connInfo.getAddress().toString().c_str());

    pClient->disconnect();
//    devData.opcode = 0xff;
    fsm.requests = 0x80000000;  // delete all pending command requests
    Serial.printf("client connection to server disconnected.\n");

    BLEDevice::startAdvertising();
    Serial.printf("advertising restarted.\n");
  }
} serverCallbacks;

class characteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue)
  {

    std::string str  = "Client";
    if (connInfo.getConnHandle()) {str += " '"; str += connInfo.getConnHandle(); str += "'";}
    str += " "; str += connInfo.getAddress().toString();
    if (subValue == 0) {
      str += " unsubscribed from ";
    } else if (subValue == 1) {
      str += " subscribed to notifications for ";
    } else if (subValue == 2) {
      str += " subscribed to indications for ";
    } else if (subValue == 3) {
      str += " subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID());
    Serial.printf("%s\n", str.c_str());

    if (pCharacteristic->getUUID().equals(BLEUUID((uint16_t)ESP_GATT_UUID_INDOOR_BIKE_DATA_CHAR)))
    {
      switch (subValue)
      {
        case 0: // unsubscribe
        case 2: // subscribe indications
          doNotifyIndoorBikeData = false;
          break;
        case 1: // subscribe notifications
        case 3: // subscribe notifications and indications
          doNotifyIndoorBikeData = true;
          break;
        default:
          doNotifyIndoorBikeData = false;
      }
    }

    if (pCharacteristic->getUUID().equals(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_STATUS_CHAR)))
    {
      switch (subValue)
      {
        case 0: // unsubscribe
        case 2: // subscribe indications
          doNotifyFitnessMachineStatus = false;
          break;
        case 1: // subscribe notifications
        case 3: // subscribe notifications and indications
          doNotifyFitnessMachineStatus = true;
          break;
        default:
          doNotifyFitnessMachineStatus = false;
      }
    }

    if (pCharacteristic->getUUID().equals(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR)))
    {
      switch (subValue)
      {
        case 0: // unsubscribe
        case 1: // subscribe notifications
          doIndicateFitnessMachineControlPoint = false;
          break;
        case 2: // subscribe indications
        case 3: // subscribe notifications and indications
          doIndicateFitnessMachineControlPoint = true;
          break;
        default:
          doIndicateFitnessMachineControlPoint = false;
      }
    }
  }

  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
  {
    if (pCharacteristic->getUUID().equals(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR)))
    {
      uint8_t numBytes = pCharacteristic->getLength();

      const uint8_t *procedure = pCharacteristic->getValue().data();
      fsm.requests |= 1 << procedure[0];    // set request bit according to opcode
      Serial.printf(" FMCP onWrite: len %d len %d proc 0x", numBytes, pCharacteristic->getValue().length()); for (uint8_t i=0; i<numBytes; i++) Serial.printf("%02x", procedure[i]); Serial.printf(" fsm.requests 0x%08x\n", (unsigned int)fsm.requests);

      switch (procedure[0])   // FMCP opcode
      {
        case FMCP_OPCODE_SET_TARGET_POWER:
          ctrlData.requestedPower = procedure[1] + 256*procedure[2];
          Serial.printf(" FMCP onWrite: set target power to %d W\n", ctrlData.requestedPower);
          break;
        default:  // not yet implemented or invalid
          // do nothing;
          break;
      }
    }
  }
} characteristicCallbacks;

class scanCallbacks : public NimBLEScanCallbacks
{
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice)
  {
    if (advertisedDevice->haveServiceUUID())
    {
      if (advertisedDevice->isAdvertisingService(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_SVC)))
      {
        std::string devName = advertisedDevice->getName();
        if (devName.compare("TC174")==0)
        {
          Serial.printf("  scan: FTMS device found: '%s'\n", devName.c_str());
          NimBLEDevice::getScan()->stop();
          foundDevice=true;
          device = advertisedDevice;  // save device reference in global for further use
        }
      }
    }
  }

  void onScanEnd(const NimBLEScanResults& results, int reason)
  {
    Serial.printf("scan end, reason %d, device count %d, restarting scan...\n", reason, results.getCount());
    NimBLEDevice::getScan()->start(scanTimeMs, false, true);
  }
} scanCallbacks;

/* notify callback  - we got a notification or indication from the connected peer device server */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
/*
    std::string str  = (isNotify == true) ? "Notification" : "Indication";
    str             += " from ";
    str             += pRemoteCharacteristic->getClient()->getPeerAddress().toString();
    str             += ": Service = " + pRemoteCharacteristic->getRemoteService()->getUUID().toString();
    str             += ", Characteristic = " + pRemoteCharacteristic->getUUID().toString();
    str             += ", Value = 0x";
    Serial.printf("%s", str.c_str());
    for (uint8_t i=0; i<pRemoteCharacteristic->getLength(); i++) Serial.printf("%02x", pData[i]);
    Serial.printf("\n");
*/
  NimBLEUUID svcUUID = pRemoteCharacteristic->getRemoteService()->getUUID();
  NimBLEUUID charUUID = pRemoteCharacteristic->getUUID();

  bool knownSvc = false;
  bool knownChar = false;
  if (svcUUID.equals(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_SVC)))
  {
    knownSvc = true;

    if (charUUID.equals(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR)))
    {
      knownChar = true;
      fitnessMachineControlPointNotificationValue[0] = 0xff;  // invalidate response field
      for (uint8_t i=0; i<length; i++) fitnessMachineControlPointNotificationValue[i] = pData[i];
    }

    if (charUUID.equals(BLEUUID((uint16_t)ESP_GATT_UUID_CROSS_TRAINER_DATA_CHAR)))
    {
      uint16_t temp;

      knownChar = true;

      uint16_t flags = pData[0] + 256*pData[1];

      uint8_t pDataPtr=2;   // first param at index 2 of pData

      // additional empty byte ??
      pDataPtr += 1;

      if (!(flags & 0x0001))  // inverse logic: more data vs instantaneous speed
      {
        devData.instSpeed = pData[pDataPtr] + 256*pData[pDataPtr+1];
//        Serial.printf("  inst speed %04x %d (%.2f km/h)\n", devData.instSpeed, devData.instSpeed, (float)devData.instSpeed / 100.0);
        pDataPtr += 2;
      }

      if (flags & 0x0002)   // average speed
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0004)   // total distance
      {
        //
        pDataPtr += 3;  // 24 bits
      }
      if (flags & 0x0008)   // step count
      {
        temp = pData[pDataPtr] + 256*pData[pDataPtr+1];
        if (temp != 0xffff)
        {
          devData.instCadence = temp * 2;   // defined in struct as 0.5 steps per minute; here as 1 step per minute
//          Serial.printf("  steps rate %04x %d (%.1f steps/min).\n", temp, temp, (float)devData.instCadence / 2.0);
        }
        pDataPtr += 2;
        temp = pData[pDataPtr] + 256*pData[pDataPtr+1];
        if (temp != 0xffff)
        {
//          Serial.printf("  average step rate %04x %d (ignored).\n", temp, temp);
        }
        pDataPtr += 2;
      }
      if (flags & 0x0010)   // stride count
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0020)   // elevation gain
      {
        //
        pDataPtr += 4;  // pos. elev gain, neg. elev gain
      }
      if (flags & 0x0040)   // inclination setting
      {
        //
        pDataPtr += 4;  // inclination, ramp angle
      }
      if (flags & 0x0080)   // resistance level
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0100)   // instantaneous power
      {
        devData.instPower = pData[pDataPtr] + 256*pData[pDataPtr+1];
//        Serial.printf("  inst power %04x %d (%d W)\n", devData.instPower, devData.instPower, devData.instPower);
        pDataPtr += 2;
      }
      if (flags & 0x0200)   // average power
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0400)   // expended energy
      {
        //
        pDataPtr += 5;  // total energy, energy per hour, energy per minute
      }
      if (flags & 0x0800)   // heart rate
      {
        devData.heartRate = pData[pDataPtr];
//        Serial.printf("  heart rate %02x %d (%d bpm)\n", devData.heartRate, devData.heartRate, devData.heartRate);
        pDataPtr += 1;  // 8 bits
      }
      if (flags & 0x1000)   // metabolic equivalent
      {
        //
        pDataPtr += 1;  // 8 bits
      }
      if (flags & 0x2000)   // elapsed time
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x4000)   // remaining time
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x8000)   // movement direction
      {
        // bit set: backwards
      }
      else
      {
        // bit cleared: forwards
      }
    }

    if (charUUID.equals(BLEUUID((uint16_t)ESP_GATT_UUID_INDOOR_BIKE_DATA_CHAR)))
    {
      knownChar = true;

      uint16_t flags = pData[0] + 256*pData[1];

      uint8_t pDataPtr=2;   // first param at index 2 of pData

      if (!(flags & 0x0001))  // inverse logic: more data vs instantaneous speed
      {
        devData.instSpeed = pData[pDataPtr] + 256*pData[pDataPtr+1];
//        Serial.printf("  inst speed %04x %d (%.2f km/h)\n", devData.instSpeed, devData.instSpeed, (float)devData.instSpeed / 100.0);
        pDataPtr += 2;
      }

      if (flags & 0x0002)   // average speed
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0004)   // instantaneous cadence
      {
        devData.instCadence = pData[pDataPtr] + 256*pData[pDataPtr+1];
//        Serial.printf("  inst cadence %04x %d (%.1f steps/min)\n", devData.instCadence, devData.instCadence, (float)devData.instCadence / 2.0);
        pDataPtr += 2;
      }
      if (flags & 0x0008)   // average cadence
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0010)   // total distance
      {
        //
        pDataPtr += 3;  // 24 bits
      }
      if (flags & 0x0020)   // resistance level
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0040)   // instantaneous power
      {
        devData.instPower = pData[pDataPtr] + 256*pData[pDataPtr+1];
//        Serial.printf("  inst power %04x %d (%d W)\n", devData.instPower, devData.instPower, devData.instPower);
        pDataPtr += 2;
      }
      if (flags & 0x0080)   // average power
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0100)   // expended energy
      {
        //
        pDataPtr += 5;  // total energy, energy per hour, energy per minute
      }
      if (flags & 0x0200)   // heart rate
      {
        devData.heartRate = pData[pDataPtr];
//        Serial.printf("  heart rate %02x %d (%d bpm)\n", devData.heartRate, devData.heartRate, devData.heartRate);
        pDataPtr += 1;  // 8 bits
      }
      if (flags & 0x0400)   // metabolic equivalent
      {
        //
        pDataPtr += 1;  // 8 bits
      }
      if (flags & 0x0100)   // elapsed time
      {
        //
        pDataPtr += 2;
      }
      if (flags & 0x0100)   // remaining time
      {
        //
        pDataPtr += 2;
      }
    }
  }

  if (!knownSvc)
    Serial.printf("%s: unimplemented service %s\n", isNotify ? "notification" : "indication", svcUUID.toString().c_str());
  if (!knownChar)
    Serial.printf("%s from svc %s: unimplemented char %s\n", isNotify ? "notification" : "indication", svcUUID.toString().c_str(), charUUID.toString().c_str());

/*
  if (charUUID.equals(BLEUUID((uint16_t)ESP_GATT_UUID_CROSS_TRAINER_DATA_CHAR)) || 
      charUUID.equals(BLEUUID((uint16_t)ESP_GATT_UUID_INDOOR_BIKE_DATA_CHAR)))
  {
//    Serial.printf("device data: inst_speed %.2f inst_power %d inst_cadence %.2f step_count %d heart_rate %d\n", (float)devData.instSpeed / 100.0, devData.instPower, (float)devData.instCadence / 2.0, devData.stepCount, devData.heartRate);
    Serial.printf("device data: inst_speed %.2f km/h   inst_power %d W   inst_cadence %.2f steps/min   heart_rate %d bpm\n", (float)devData.instSpeed / 100.0, devData.instPower, (float)devData.instCadence / 2.0, devData.heartRate);
  }
*/
}

bool connectToServer()
  /*
   * - create client structures for `device` found by scanning
   * - connect to server
   * - check for service: fitness machine
   * - check for and subscribe to characteristic: fitness machine control point
   * - check for and read out characteristic: resistance level range (and fill struct `devInfo`)
   * - check for and subscribe to characteristic: cross trainer data
   * - check for and subscribe to characteristic: indoor bike data
   */
{
  Serial.printf("connecting to server...\n");
  if (pClient && pClient->isConnected())
  {
    Serial.printf("  already connected.\n");
    return true;
  }

  pClient = nullptr;

  /** Check if we have a client we should reuse first **/
  if (NimBLEDevice::getCreatedClientCount())
  {
    Serial.printf("  we already have stored client information, do we already know the peer device?\n");
    /**
      *  Special case when we already know this device, we send false as the
      *  second argument in connect() to prevent refreshing the service database.
      *  This saves considerable time and power.
      */
    pClient = NimBLEDevice::getClientByPeerAddress(device->getAddress());
    if (pClient)
    {
      if (!pClient->connect(device, false))
      {
        Serial.printf("    peer device already known, but reconnect failed\n");
        return false;
      }
      Serial.printf("    peer device known, reconnected\n");
    }
    else
    {
      Serial.printf("    no, we don't yet know the peer device\n");
      /**
        *  We don't already have a client that knows this device,
        *  check for a client that is disconnected that we can use.
        */
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  /** No client to reuse? Create a new one. */
  if (!pClient)
  {
    Serial.printf("  no client to be reused for this peer device, we need to create a new one\n");
    if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS)
    {
      Serial.printf("  max clients reached - no more connections available\n");
      return false;
    }

    pClient = NimBLEDevice::createClient();

    Serial.printf("  new client created\n");

//    pClient->setClientCallbacks(&clientCallbacks, false);
    /**
      *  Set initial connection parameters:
      *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
      *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
      *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 150 * 10ms = 1500ms timeout
      */
    pClient->setConnectionParams(12, 12, 0, 150);

    /** Set how long we are willing to wait for the connection to complete (milliseconds), default is 30000. */
    pClient->setConnectTimeout(5 * 1000);

    if (!pClient->connect(device)) {
      /** Created a client but failed to connect, don't need to keep it as it has no data */
      NimBLEDevice::deleteClient(pClient);
      Serial.printf("  failed to connect to peer device, client deleted\n");
      return false;
    }
    Serial.printf("  connected to peer device\n");
  }

  if (!pClient->isConnected())
  {
    if (!pClient->connect(device))
    {
//      NimBLEDevice::deleteClient(pClient);
      Serial.printf("  failed to (re-)connect to peer device\n");
      return false;
    }
  }

  Serial.printf("connected to: %s RSSI: %d\n", pClient->getPeerAddress().toString().c_str(), pClient->getRssi());

  /** Now we can read/write/subscribe the characteristics of the services we are interested in */
  NimBLERemoteService*        pSvc = nullptr;
  NimBLERemoteCharacteristic* pChr = nullptr;
//  NimBLERemoteDescriptor*     pDsc = nullptr;

  pSvc = pClient->getService(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_SVC));
  if (pSvc)
  {
    Serial.printf("found service: fitness machine\n");
  }
  else
  {
    Serial.printf("service 0x%04x not found\n", ESP_GATT_UUID_FITNESS_MACHINE_SVC);
    pClient->disconnect();
    return false;
  }

  pChr = pSvc->getCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR));
  if (pChr)
  {
    Serial.printf("  found characteristic: fitness machine control point\n");
    if (pChr->canNotify())
    {
      if (!pChr->subscribe(true, notifyCB))   // fail, if notifiable, but not subscribable
      {
        Serial.printf("    notifiable, but subscribing failed\n");
        pClient->disconnect();
        return false;
      }
      else
        Serial.printf("    subscribed.\n");
    }    
  }
  else
  {
    Serial.printf("  characteristic 0x%04x not found\n", ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR);
    pClient->disconnect();
    return false;
  }

  pChr = pSvc->getCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_SUPPORTED_RESISTANCE_LEVEL_RANGE_CHAR));
  if (pChr)
  {
    Serial.printf("  found characteristic: supported resistance level range\n");
    if (pChr->canRead())
    {
      const char *val =  pChr->readValue().c_str();
      if (pChr->getLength() != 6)    // getLength() only returns valid length AFTER readValue()
      {
        Serial.printf("    resistance level range value not 6 bytes long\n");
        pClient->disconnect();
        return false;
      }
      devInfo.RLR_low = (float)(val[0] + 256*val[1]) / 10.0;
      devInfo.RLR_high = (float)(val[2] + 256*val[3]) / 10.0;
      devInfo.RLR_step = (float)(val[4] + 256*val[5]) / 10.0;
      Serial.printf("    resistance levels: low %.1f high %.1f stepsize %.1f\n", devInfo.RLR_low, devInfo.RLR_high, devInfo.RLR_step);
    }
  }
  else
  {
    Serial.printf("  characteristic 0x%04x not found\n", ESP_GATT_UUID_SUPPORTED_RESISTANCE_LEVEL_RANGE_CHAR);
    pClient->disconnect();
    return false;
  }

  pChr = pSvc->getCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_CROSS_TRAINER_DATA_CHAR));
  if (pChr) 
  {
    Serial.printf("  found characteristic: cross trainer data\n");
    if (pChr->canNotify())
    {
      if (!pChr->subscribe(true, notifyCB))   // fail, if notifiable, but not subscribable
      {
        Serial.printf("    notifiable, but subscribing failed\n");
        pClient->disconnect();
        return false;
      }
      else
        Serial.printf("    subscribed.\n");
    }
  }
  else
  {
    Serial.printf("  characteristic 0x%04x not found\n", ESP_GATT_UUID_CROSS_TRAINER_DATA_CHAR);
    pClient->disconnect();
    return false;
  }

  pChr = pSvc->getCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_INDOOR_BIKE_DATA_CHAR));
  if (pChr) 
  {
    bool subscribed = false;
    Serial.printf("  found characteristic: indoor bike data\n");
    if (pChr->canNotify())
    {
      if (!pChr->subscribe(true, notifyCB))   // fail, if notifiable, but not subscribable
      {
        Serial.printf("    notifiable, but subscribing failed.\n");
      }
      else
      {
        subscribed = true;
        Serial.printf("    subscribed to notification.\n");
      }
    }
    else
      Serial.printf ("    not notifiable.\n");

    if (!subscribed)
    {
      if (pChr->canIndicate())
      {
        if (!pChr->subscribe(false, notifyCB))
        {
          Serial.printf("    indicatable, but subscribing failed.\n");
        }
        else
        {
          subscribed = true;
          Serial.printf("    subscribed to indication.\n");
        }
      }
      else
        Serial.printf ("    not indicatable.\n");
    }
  }
  else
  {
    Serial.printf("  characteristic 0x%04x not found\n", ESP_GATT_UUID_INDOOR_BIKE_DATA_CHAR);
    pClient->disconnect();
    return false;
  }

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);  // wait 2 s for the serial i/f to initialise

  NimBLEDevice::init("TrainerProxy\0");

  /*
   * server
   */
  Serial.printf("creating server:\n");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  /* FTMS_v1.0.pdf, p. 17,18:
   *  Where a characteristic can be notified or indicated, a Client Characteristic Configuration descriptor shall be included in that characteristic as required by the Core Specification [1].
   *  FitnessMachineFeature, mandatory, read
   *  Indoor Bike Data, optional, notify
   *  Fitness Machine Control Point, optonal, write/indicate
   *  Supported Inclination Range, optional, read, mandatory if inclination target setting feature is supported
   *  Supported Resistance Level Range, optional, read, mandatory if resistance target setting feature is supported
   *  Fitness Machine Status, optional, notify, mandatory if fitness machine control point is supported
   */
  NimBLEService *pFitnessMachineService = pServer->createService(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_SVC));

  NimBLECharacteristic *pCharFitnessMachineFeature = pFitnessMachineService->createCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_FEATURE_CHAR), NIMBLE_PROPERTY::READ);
//  const uint32_t FitnessMachineFeature[] = {0x00004083, 0x0000000f};    // FTMS p.20     measurements: speed, cadence, resistance, power; targets: speed, inclication, resistance, power
//  const uint32_t FitnessMachineFeature[] = {0x00004083, 0x00000008};    // FTMS p.20     measurements: speed, cadence, resistance, power; targets: power
  const uint32_t FitnessMachineFeature[] = {0x00004083, 0x0001020f};    // FTMS p.20     measurements: speed, cadence, resistance, power; targets: speed, inclication, resistance, power, training time, cadence
  pCharFitnessMachineFeature->setValue((uint8_t *)&FitnessMachineFeature, 8);
  
  pCharFitnessMachineStatus = pFitnessMachineService->createCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_STATUS_CHAR), NIMBLE_PROPERTY::NOTIFY);
  pCharFitnessMachineStatus->setValue((uint8_t *)&fitnessMachineStatus, 1);
  pCharFitnessMachineStatus->setCallbacks(&characteristicCallbacks);

  pCharIndoorBikeData = pFitnessMachineService->createCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_INDOOR_BIKE_DATA_CHAR), NIMBLE_PROPERTY::NOTIFY);
  pCharIndoorBikeData->setValue((uint8_t *)&indoorBikeData, 8); // 4 x uint16_t = 8 bytes
  pCharIndoorBikeData->setCallbacks(&characteristicCallbacks);

  pCharFitnessMachineControlPoint = pFitnessMachineService->createCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR), NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);  // spec says to indicate (FTMS v1.0 p. 64), but that does not generate a reply from the mobile app
  pCharFitnessMachineControlPoint->setValue((uint8_t *)&fitnessMachineControlPoint, 3);
  pCharFitnessMachineControlPoint->setCallbacks(&characteristicCallbacks);

  pFitnessMachineService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("TrainerProxy");
  // advertising data types: Core Specification Supplement v6 https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=302735
  // FTMS ch. 3
  std::string advertisingServiceData = "\x16\x26\x18\x01\x20\x00";  // 0x16 Service Data AD Type (16bit UUID), UUID, flags, machine type field 
  NimBLEAdvertisementData advData;
  advData.setServiceData(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_SVC), advertisingServiceData);
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->addServiceUUID(pFitnessMachineService->getUUID());
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();
  Serial.printf("  done.\n");

  /*
   * scan for client
   */
  Serial.printf("starting scan:");
  NimBLEScan* pScan = NimBLEDevice::getScan();

  /** Set the callbacks to call when scan events occur, no duplicates */
  pScan->setScanCallbacks(&scanCallbacks, false);

  /** Set scan interval (how often) and window (how long) in milliseconds */
  pScan->setInterval(100);
  pScan->setWindow(100);

  pScan->setActiveScan(true);

  Serial.flush();
  pScan->start(scanTimeMs);
  Serial.printf("  scanning...\n");

  /*
   * initialise machine state
   */
  fsm.state = FSM_UNDEFINED;
}


void waitForFCMPNotification(uint8_t opcode)
{
  // TODO: timeout and bool reply
//  Serial.printf("waiting for reply on opcode %d...\n", opcode);
  while (!(fitnessMachineControlPointNotificationValue[0] == 0x80 &&    // response opcode
           fitnessMachineControlPointNotificationValue[1] == opcode &&  // request opcode
           fitnessMachineControlPointNotificationValue[2] == 0x01))     // result: 01..success
  {
//    Serial.printf("fmcp notification: waiting for opcode %d, value received from server 0x", opcode);for (uint8_t i=0; i<3; i++) Serial.printf("%02x", fitnessMachineControlPointNotificationValue[i]);Serial.printf("\n");
    delay(500);
  }
//  Serial.printf("fmcp notification: success received from server for opcode %d\n", opcode);
}


void loop()
{
  if (doConnectToServer)
  {
    if (connectToServer())
    {
      doConnectToServer=false;
      Serial.printf("client connection to server (re-)established, notifications should come in now...\n");
    }
    else
    {
      Serial.printf("failed to connect to peer device server, retrying...\n");
    }
  }

  NimBLERemoteService *pSvc = nullptr;
  NimBLERemoteCharacteristic *pChr = nullptr;
  if (pClient && pClient->isConnected()) pSvc = pClient->getService(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_SVC));
  if (pSvc) pChr = pSvc->getCharacteristic(BLEUUID((uint16_t)ESP_GATT_UUID_FITNESS_MACHINE_CONTROL_POINT_CHAR));
  uint8_t writeValue[11];

  // fsm
  uint8_t fsmOldState = fsm.state;
  uint32_t fsmRequestSetATarget = fsm.requests & 0x0001fe7c;
  fitnessMachineControlPointNotificationValue[0] = 0xff;    // invalidate return codes
  switch (fsm.state)
  {
    case FSM_UNDEFINED:
      fsm.requests &= ~(1<<31);
      if (fsm.requests) fsm.state = FSM_CTRL_REQUEST; // if ANY request bit is set, continue
      break;
    case FSM_CTRL_REQUEST:
      if (fsm.requests & (1<<31)) // client disconnect
        fsm.state = FSM_UNDEFINED;
      else if (pChr && pChr->canWrite())
      {
        // TODO error handling: writeValue, notification receive timeout, notification send failure
        writeValue[0] = FMCP_OPCODE_REQUEST_CONTROL;
        pChr->writeValue((uint8_t *)&writeValue[0], 1, true);
        waitForFCMPNotification(writeValue[0]);
        if (fsm.requests & (1<<writeValue[0])) // only notify if bit has been set by FMCP
        {
          fitnessMachineControlPoint[1]=writeValue[0];
          fitnessMachineControlPoint[2]=0x01; // success
          pCharFitnessMachineControlPoint->setValue((uint8_t *)&fitnessMachineControlPoint,3);
          pCharFitnessMachineControlPoint->notify();  // indicate() does not generate further replies from the client aka mobile app
          fsm.requests &= ~(1<<writeValue[0]);
        }
        fsm.state = FSM_CTRL_REQUESTED;
      }
      break;
    case FSM_CTRL_REQUESTED:
      if (fsmRequestSetATarget || fsm.requests & 0x00000182)   //  set a target, or: reset, stop, start
        fsm.state = FSM_RESET_REQUEST;
      if (fsm.requests & 0x80000000)    // client disconnect
        fsm.state = FSM_UNDEFINED;
      break;
    case FSM_RESET_REQUEST:
      if (fsm.requests & 0x80000000)    // client disconnect
        fsm.state = FSM_UNDEFINED;
      else if (pChr && pChr->canWrite())
      {
        // TODO error handling: writeValue, notification receive timeout, notification send failure
        writeValue[0] = FMCP_OPCODE_RESET;
        pChr->writeValue((uint8_t *)&writeValue[0], 1, true);
        waitForFCMPNotification(writeValue[0]);
        if (fsm.requests & (1<<writeValue[0])) // only notify if bit has been set by FMCP
        {
          fitnessMachineControlPoint[1]=writeValue[0];
          fitnessMachineControlPoint[2]=0x01; // success
          pCharFitnessMachineControlPoint->setValue((uint8_t *)&fitnessMachineControlPoint,3);
          pCharFitnessMachineControlPoint->notify();  // indicate() does not generate further replies from the client aka mobile app
          fsm.requests &= ~(1<<writeValue[0]);
        }
        fsm.state = FSM_RESET_DONE;
      }
      break;
    case FSM_RESET_DONE:
      if (fsmRequestSetATarget || fsm.requests & 0x00000180)  // set a target, or: stop, start
        fsm.state = FSM_STOP_REQUEST;
      if (fsm.requests & 0x80000000)    // client disconnect
        fsm.state = FSM_UNDEFINED;
      break;
    case FSM_STOP_REQUEST:
      if (fsm.requests & 0x80000000)    // client disconnect
        fsm.state = FSM_UNDEFINED;
      else if (pChr && pChr->canWrite())
      {
        // TODO error handling: writeValue, notification receive timeout, notification send failure
        writeValue[0] = FMCP_OPCODE_STOP_PAUSE;
        pChr->writeValue((uint8_t *)&writeValue[0], 1, true);
        waitForFCMPNotification(writeValue[0]);
        if (fsm.requests & (1<<writeValue[0])) // only notify if bit has been set by FMCP
        {
          fitnessMachineControlPoint[1]=writeValue[0];
          fitnessMachineControlPoint[2]=0x01; // success
          pCharFitnessMachineControlPoint->setValue((uint8_t *)&fitnessMachineControlPoint,3);
          pCharFitnessMachineControlPoint->notify();  // indicate() does not generate further replies from the client aka mobile app
          fsm.requests &= ~(1<<writeValue[0]);
        }
        fsm.state = FSM_STOP_DONE;
      }
      break;
    case FSM_STOP_DONE:
      if (fsmRequestSetATarget || fsm.requests & 0x00000080)  // set a target, or: start
        fsm.state = FSM_START_REQUEST;
      if (fsm.requests & 0x80000000)  // client disconnect
        fsm.state = FSM_UNDEFINED;
      break;
    case FSM_START_REQUEST:
      if (fsm.requests & 0x80000000)  // client disconnect
        fsm.state = FSM_UNDEFINED;
      else if (pChr && pChr->canWrite())
      {
        // TODO error handling: writeValue, notification receive timeout, notification send failure
        writeValue[0] = FMCP_OPCODE_START_RESUME;
        pChr->writeValue((uint8_t *)&writeValue[0], 1, true);
        waitForFCMPNotification(writeValue[0]);
        if (fsm.requests & (1<<writeValue[0])) // only notify if bit has been set by FMCP
        {
          fitnessMachineControlPoint[1]=writeValue[0];
          fitnessMachineControlPoint[2]=0x01; // success
          pCharFitnessMachineControlPoint->setValue((uint8_t *)&fitnessMachineControlPoint,3);
          pCharFitnessMachineControlPoint->notify();  // indicate() does not generate further replies from the client aka mobile app
          fsm.requests &= ~(1<<writeValue[0]);
        }
        fsm.state = FSM_START_DONE;
      }
      break;
    case FSM_START_DONE:
      if (fsmRequestSetATarget)
        fsm.state = FSM_SET_TARGET_REQUEST;
      if (fsm.requests & 0x00000010)  // stop
        fsm.state = FSM_STOP_REQUEST;
      if (fsm.requests & 0x00000002)  // reset
        fsm.state = FSM_RESET_REQUEST;
      if (fsm.requests & 0x80000000)  // client disconnect
        fsm.state = FSM_UNDEFINED;
      break;
    case FSM_SET_TARGET_REQUEST:
      if (fsm.requests & 0x80000000)  // client disconnect
        fsm.state = FSM_UNDEFINED;
      else if (pChr && pChr->canWrite())
      {
        // TODO error handling: writeValue, notification receive timeout, notification send failure
        writeValue[0] = 0;
        int8_t requestedOpcode = -1;
        if (fsm.requests & (1<<FMCP_OPCODE_SET_TARGET_POWER))
        {
          requestedOpcode = FMCP_OPCODE_SET_TARGET_POWER;
          float numResistanceSteps = ((float)devInfo.RLR_high - devInfo.RLR_low) / devInfo.RLR_step;
          ctrlData.commandedResistance = (uint8_t)round((float)ctrlData.requestedPower * numResistanceSteps / 350);
          writeValue[0] = FMCP_OPCODE_SET_TARGET_RESISTANCE;
          writeValue[1] = ctrlData.commandedResistance * 10;  // resolution: 0.1 steps
          pChr->writeValue((uint8_t *)&writeValue, 2, true);
          waitForFCMPNotification(FMCP_OPCODE_SET_TARGET_RESISTANCE);
          Serial.printf("  requested power %d resistance level set %d\n", ctrlData.requestedPower, ctrlData.commandedResistance);
        }
        if (requestedOpcode>0)
        {
          if (fsm.requests & (1<<requestedOpcode)) // only notify if bit has been set by FMCP
          {
            fitnessMachineControlPoint[1]=requestedOpcode;
            fitnessMachineControlPoint[2]=0x01; // success
            pCharFitnessMachineControlPoint->setValue((uint8_t *)&fitnessMachineControlPoint,3);
            pCharFitnessMachineControlPoint->notify();  // indicate() does not generate further replies from the client aka mobile app
            fsm.requests &= ~(1<<requestedOpcode);
          }
        }
        fsm.state = FSM_SET_TARGET_DONE;
      }
      break;
    case FSM_SET_TARGET_DONE:
      if (fsm.requests & 0x80000000)  // client disconnect
        fsm.state = FSM_UNDEFINED;
      else if (fsm.requests & 0x00000002) // reset
        fsm.state = FSM_RESET_REQUEST;
      else if (fsm.requests & 0x00000010)  // stop
        fsm.state = FSM_STOP_REQUEST;
      else
        fsm.state = FSM_START_REQUEST;  // all other cases
      break;
    default:
      fsm.state = FSM_UNDEFINED;
  }
//  if (fsmOldState != fsm.state)
//    Serial.printf("FSM: new state: %d\n", fsm.state);

  // report back to the client (aka mobile app)
  if (doNotifyIndoorBikeData)
  {
    indoorBikeData[1] = devData.instSpeed;  // inst. speed [0.01 km/h]
    indoorBikeData[2] = devData.instCadence;// inst. cadence [0.5 rpm]
    indoorBikeData[3] = devData.instPower;  // inst. power [W]
    pCharIndoorBikeData->setValue((uint8_t *)&indoorBikeData, 8);
    pCharIndoorBikeData->notify();
  }
/*
  if (doNotifyFitnessMachineStatus)
  {
    fitnessMachineStatus[0] = devData.opcode;
    pCharFitnessMachineStatus->setValue((uint8_t *)&fitnessMachineStatus, 1);
    pCharFitnessMachineStatus->notify();
  }
*/
  delay(100);
}