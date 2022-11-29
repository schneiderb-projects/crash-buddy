/*
Crash Buddy: impact detection
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include "RingBuffer.h"

//sensor reading/crash detection related variables
//#define DEFAULT_THRESHHOLD 0
#define DEFAULT_THRESHHOLD 100
volatile uint16_t new_data = 0;
volatile uint16_t total_new_data = 0;
bool crash_detected = false;

//BLE Gatt Server UUIDS, all letters must be lowercase
#define PREAMBLE "0998"
#define DEVICE_UUID "-1280-49a1-bacf-965209262e66"  // last 14 bytes of char uuid
//#define DEVICE_UUID  "-1280-49A1-BACF-965209262E66" // last 14 bytes of char uuid

//UUIDS for service and characteristics
#define SERVICE_UUID PREAMBLE "0000" DEVICE_UUID
#define CHARACTERISTIC_UUID_STATUS PREAMBLE "0001" DEVICE_UUID
#define CHARACTERISTIC_UUID_SET_THRESHHOLD PREAMBLE "0003" DEVICE_UUID
#define CHARACTERISTIC_UUID_DATA_AVAILABLE PREAMBLE "0004" DEVICE_UUID
#define CHARACTERISTIC_UUID_DATA_CHARS PREAMBLE "0005" DEVICE_UUID
#define CHARACTERISTIC_UUID_DATA_SIZE PREAMBLE "0006" DEVICE_UUID
#define CHARACTERISTIC_UUID_SET_ENABLE_DEBUG PREAMBLE "0007" DEVICE_UUID
#define CHARACTERISTIC_UUID_CRASH_DATA_CHAR_SIZE PREAMBLE "0008" DEVICE_UUID

//crash data characteristics: must edit these by hand to change the number of datapoints sent
#pragma pack(1)      // prevent compiler adding padding for 32 bit alignment
struct data_point {  // total size = 6 bytes
  int16_t value;     // value recorded by accelerometer
  int32_t time;      // clock time, not real time
};

#define CHARACTERISTIC_UUID_CRASH_DATA_INITIAL 0x0010  //initial address for Crash Data Characteristics
#define TOTAL_CRASH_DATAPOINTS 2000                    // Total number of crash data datapoints
#define SIZEOF_DATA_POINT sizeof(struct data_point)    // sizeof(struct data_point)
#define MAX_BYTES_PER_CHAR 516
#define DATA_POINTS_PER_CHAR (MAX_BYTES_PER_CHAR / SIZEOF_DATA_POINT)       // Datapoints stored in one characteristic
#define CRASH_DATA_CHARS (TOTAL_CRASH_DATAPOINTS / DATA_POINTS_PER_CHAR + 2)  // number of crash data characteristics

#define UUID_COUNT (9 + CRASH_DATA_CHARS)

//BLE GATT server
BLEServer* pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

//BLE GATT characteristics
BLECharacteristic* pStatusCharacteristic;
BLECharacteristic* pDataAvailableCharacteristic;
BLECharacteristic* pDataSizeCharacteristic;
BLECharacteristic* pSetThreshholdCharacteristic;
BLECharacteristic* pSetEnableDebugCharacteristic;
BLECharacteristic* pCrashDataCharSizeCharacteristic;
BLECharacteristic* pCrashDataCharsCharacteristic;
BLECharacteristic* pCrashDataCharacteristics[CRASH_DATA_CHARS];

//BLE GATT characteristic values
uint32_t statusValue = 0;
uint32_t threshholdValue = DEFAULT_THRESHHOLD;
uint8_t* setEnableDebugValue;

//data storage related variables
struct data_point data_buffer[TOTAL_CRASH_DATAPOINTS] = { 0 };

// Should fake data be used?

struct data_point temp_data = { 0, 0 };
//#define USE_FAKE_DATA // Define to use fake data instead of reading from the sensor
//#define USE_FAKE_BATCHED_DATA // Must also define use USE_FAKE_DATA otherwise this is ignored
#ifdef USE_FAKE_DATA
// fake data variables
hw_timer_t* timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#else
// real data variables
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
long last_read = 0;
#endif

RingBuffer rb(data_buffer, TOTAL_CRASH_DATAPOINTS * sizeof(struct data_point), sizeof(struct data_point));

void sensor_read();  // sensor read declaration

//BLE Gatt Table Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    if (pCharacteristic->getValue().length() > 0) {
      Serial.print("*********");
      Serial.write(pCharacteristic->getUUID().toString().c_str());
      Serial.println("*********");
      Serial.print("********* Length: ");
      Serial.print(pCharacteristic->getValue().length());
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < pCharacteristic->getValue().length(); i++)
        Serial.print((uint8_t) pCharacteristic->getValue()[i]); Serial.print(" ");

      Serial.println();
      Serial.println("*********");

      if (pCharacteristic->getUUID().toString() == CHARACTERISTIC_UUID_SET_THRESHHOLD) {
        threshholdValue = 0;
        threshholdValue += pCharacteristic->getData()[3]; 
        threshholdValue += pCharacteristic->getData()[2] << 8; 

        Serial.print("New threshhold: "); Serial.println(threshholdValue);
      }

      else if (pCharacteristic->getUUID().toString() == CHARACTERISTIC_UUID_SET_ENABLE_DEBUG) {
        setEnableDebugValue = pCharacteristic->getData();
      }
    }
  }
};

void create_characteristics(BLEService* pService) {
  // status
  pStatusCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_STATUS,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pStatusCharacteristic->addDescriptor(new BLE2902());

  pStatusCharacteristic->setValue((uint8_t*)&statusValue, sizeof(statusValue));

  //data available
  pDataAvailableCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_DATA_AVAILABLE,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pDataAvailableCharacteristic->addDescriptor(new BLE2902());

  uint32_t dataAvailableValue = 0;
  pDataAvailableCharacteristic->setValue((uint8_t*)&dataAvailableValue, sizeof(dataAvailableValue));

  //data characteristic size
  pDataSizeCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_DATA_SIZE,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pDataSizeCharacteristic->addDescriptor(new BLE2902());


  uint32_t dataSizeValue = TOTAL_CRASH_DATAPOINTS;
  pDataSizeCharacteristic->setValue((uint8_t*)&dataSizeValue, sizeof(dataSizeValue));

  //set threshhold
  pSetThreshholdCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_SET_THRESHHOLD,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  pSetThreshholdCharacteristic->addDescriptor(new BLE2902());

  pSetThreshholdCharacteristic->setCallbacks(new MyCallbacks());

  uint32_t threshhold = DEFAULT_THRESHHOLD;
  pSetThreshholdCharacteristic->setValue((uint8_t*)&threshhold, sizeof(threshhold));
  //threshholdValue = *((uint32_t*)pSetThreshholdCharacteristic->getData());

  //set enable debug
  pSetEnableDebugCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_SET_ENABLE_DEBUG,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  pSetEnableDebugCharacteristic->addDescriptor(new BLE2902());

  pSetEnableDebugCharacteristic->setCallbacks(new MyCallbacks());

  uint8_t setEnableDebug = 0;
  pSetEnableDebugCharacteristic->setValue((uint8_t*)&setEnableDebug, sizeof(setEnableDebug));

  // crash data char size
  pCrashDataCharSizeCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_CRASH_DATA_CHAR_SIZE,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pCrashDataCharSizeCharacteristic->addDescriptor(new BLE2902());

  uint32_t dataCharSizeValue = DATA_POINTS_PER_CHAR;
  pCrashDataCharSizeCharacteristic->setValue((uint8_t*)&dataCharSizeValue, sizeof(dataCharSizeValue));

  // crash data chars
  pCrashDataCharsCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_DATA_CHARS,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pCrashDataCharsCharacteristic->addDescriptor(new BLE2902());

  uint32_t dataCharsValue = CRASH_DATA_CHARS;
  pCrashDataCharsCharacteristic->setValue((uint8_t*)&dataCharsValue, sizeof(dataCharsValue));

  // crash data characteristics
  char uuid[37] = { 0 };
  for (int i = 0; i < CRASH_DATA_CHARS; i++) {
    sprintf(uuid, "%s%04X%s", PREAMBLE, CHARACTERISTIC_UUID_CRASH_DATA_INITIAL + i, DEVICE_UUID);
    // crash data char
    pCrashDataCharacteristics[i] = pService->createCharacteristic(
      uuid,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    pCrashDataCharacteristics[i]->addDescriptor(new BLE2902());
  }
}


void setup() {
  Serial.begin(115200);

#ifdef USE_FAKE_DATA
  //enable a timer interrupt in place of the pin interrupt when using an actual sensor
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &sensor_read, true);
  timerAlarmWrite(timer, 500000, true);
  timerAlarmEnable(timer);
#else
  // enable sensor
  if (!accel.begin()) {
    // TODO: uh oh
  }
#endif

  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService* pService = pServer->createService(
    BLEUUID::fromString(SERVICE_UUID),
    UUID_COUNT * 3  // each characteristic gets a value char and a descriptor char so 2 per UUID
  );

  // Initialize all the characteristics
  create_characteristics(pService);

  // Start the service
  pService->start();

  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->setScanResponse(false);
  pServer->getAdvertising()->setMinPreferred(0x06);  // functions that help with iPhone connections issue

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.print("Threshhold: "); Serial.println(threshholdValue);
  Serial.println("Ready for connection...");
}

void sensor_read() {
#ifdef USE_FAKE_DATA
  // For fake data
  portENTER_CRITICAL_ISR(&mux);
#ifdef USE_FAKE_BATCHED_DATA
  int size = min((int)DATA_POINTS_PER_CHAR, TOTAL_CRASH_DATAPOINTS);
  for (int i = 0; i < size; i++) {
    temp_data.time++;
    temp_data.value = (temp_data.value + 1) % 40;
    rb.push(&temp_data);
  }

  new_data += size;
#else
  rb.push(&temp_data);
  temp_data.time++;
  temp_data.value = (temp_data.value + 1) % 20;
  new_data += 1;

#endif
  total_new_data += new_data;
  portEXIT_CRITICAL_ISR(&mux);
#else
  // actual sensor reading
  if (millis() - last_read > 10) {
    sensors_event_t event;
    accel.getEvent(&event);
    temp_data.time = (int32_t)millis();
    temp_data.value = (int16_t)(10 * sqrt(event.acceleration.x * event.acceleration.x + event.acceleration.y * event.acceleration.y + event.acceleration.z * event.acceleration.z) * 0.101971621);
    rb.push(&temp_data);
    if (temp_data.value > threshholdValue) {
      Serial.print(temp_data.time);
      Serial.print(" ");
      Serial.println(temp_data.value);
    }

    new_data += 1;
    total_new_data += 1;
    last_read = millis();
  }
#endif
}

void update_notify(BLECharacteristic* c, void* value, int num_bytes) {
  c->setValue((uint8_t*)value, num_bytes);
  c->notify();
}

void ble_notify_new_data() {
  uint8_t val = *pDataAvailableCharacteristic->getData() + 1;
  pDataAvailableCharacteristic->setValue(&val, 1);
  pDataAvailableCharacteristic->notify();
}

void crash_detect() {
  struct data_point* temp;
  for (int i = 0; i < new_data; i++) {
    temp = (struct data_point*)rb.get_at_index(TOTAL_CRASH_DATAPOINTS - new_data + i);
    if (temp->value >= threshholdValue && !crash_detected) {
      crash_detected = 1;
    }
  }
  total_new_data = 0;
  new_data = 0;
}

//TODO: fix data race to fill characteristics before next sensor interupt
void fill_data_characteristics() {
  uint index = rb.get_index() / sizeof(struct data_point);
  uint data_remaining = TOTAL_CRASH_DATAPOINTS;
  uint size;
  for (int i = 0; i < CRASH_DATA_CHARS; i++) {
    size = min(min(TOTAL_CRASH_DATAPOINTS - index, DATA_POINTS_PER_CHAR), data_remaining);
    pCrashDataCharacteristics[i]->setValue((uint8_t*)&data_buffer[index], size * sizeof(struct data_point));
    data_remaining -= size;
    index = (index + size) % TOTAL_CRASH_DATAPOINTS;
  }
  
  /*Serial.println("Sending");
  for(int i = 0; i < TOTAL_CRASH_DATAPOINTS; i++) {
    uint8_t * val = (uint8_t*)rb.get_at_index(i);
    Serial.print(i); Serial.print(": "); Serial.print(val[0], HEX); Serial.print(" ");
    Serial.print(val[1], HEX); Serial.print(" ");
    Serial.print(val[2], HEX); Serial.print(" ");  
    Serial.print(val[3], HEX); Serial.print(" ");
    Serial.print(val[4], HEX); Serial.print(" ");
    Serial.println(val[5], HEX);
  }

  for(int i = 0; i < TOTAL_CRASH_DATAPOINTS; i++) {
    struct data_point * val = (struct data_point*)rb.get_at_index(i);
    Serial.print(i); Serial.print(", time: "); Serial.print(val->time); 
    Serial.print(", value: "); Serial.println(val->value);
  }*/
}

void loop() {
  if (deviceConnected) {
  }

  if (threshholdValue != 0) {
#ifndef USE_FAKE_DATA
    sensor_read();
#endif
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    //Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  if (new_data) {
    if (!crash_detected) {
      crash_detect();

      /*for (int i = 0; i < TOTAL_CRASH_DATAPOINTS; i++) {
        Serial.print(data_buffer[i].value);
        Serial.print(" ");
        Serial.print(data_buffer[i].time);
        Serial.print(" | ");
      }
      Serial.println();*/

      if (crash_detected) {
        Serial.println("Crash Detected!");
      }
    }
    // if (crash_detected && !(total_new_data > (TOTAL_CRASH_DATAPOINTS / 2))) {
    //   Serial.print("crash_detected: "); Serial.print(crash_detected); Serial.print("total_new_data: "); Serial.println(total_new_data);
    // }
    if (crash_detected && (total_new_data > (TOTAL_CRASH_DATAPOINTS / 2))) {
      fill_data_characteristics();
      ble_notify_new_data();
      Serial.println("Crash Notified!");
      crash_detected = 0;
      total_new_data = 0;
      new_data = 0;
    }
  }
}
