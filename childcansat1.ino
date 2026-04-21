// ============================================================================
//  CHILD NODE — BSAET CanSat 2026
//  Flash to all 5 child CanSats. Change MY_ADDRESS to 1–5 for each board.
//
//  Libraries required (Sketch → Include Library → Manage Libraries):
//    - RadioHead by Mike McCauley
//    - Adafruit ADXL343 (install all dependencies)
//    - SparkFun BMP581 Arduino Library
//    - TinyGPSPlus by Mikal Hart
// ============================================================================

#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RHMesh.h>
#include <Adafruit_ADXL343.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <TinyGPSPlus.h>

// ── CHANGE THIS for each child board (1, 2, 3, 4, or 5) ──────────────────────
#define MY_ADDRESS      1
#define PARENT_ADDRESS  10

// ── Pin Definitions ───────────────────────────────────────────────────────────
#define I2C_SDA   8
#define I2C_SCL   9
#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS   10
#define LORA_DIO0 41
#define LORA_RST  40
#define GPS_RX    16
#define GPS_TX    17

#define LORA_FREQ 915.0

// ── Sensor Status Flags ───────────────────────────────────────────────────────
bool accelOK = false;
bool bmpOK   = false;
bool gpsOK   = false;

// ── Objects ───────────────────────────────────────────────────────────────────
RH_RF95 rf95(LORA_CS, LORA_DIO0);
RHMesh  mesh(rf95, MY_ADDRESS);

Adafruit_ADXL343 accel(12345);
BMP581 bmp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ── Packet Definition (shared across all sketches — must be identical) ────────
#pragma pack(1)
struct SensorPacket {
  uint8_t  nodeId;
  uint8_t  hopCount;
  float    accelX, accelY, accelZ;
  float    pressure;
  float    temperature;
  float    latitude;
  float    longitude;
  float    altitude;
  uint8_t  sensorFlags;
};
#pragma pack()

void feedGPS(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== Child Node %d ===\n", MY_ADDRESS);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (accel.begin()) {
    accel.setRange(ADXL343_RANGE_4_G);
    accelOK = true;
    Serial.println("[OK]   ADXL343 accelerometer");
  } else {
    Serial.println("[WARN] ADXL343 not found — accel fields will be NAN");
  }

  if (bmp.beginI2C() == BMP5_OK) {
    bmpOK = true;
    Serial.println("[OK]   BMP581 pressure/temp");
  } else {
    Serial.println("[WARN] BMP581 not found — pressure/temp fields will be NAN");
  }

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  feedGPS(2000);
  if (gps.charsProcessed() > 0) {
    gpsOK = true;
    Serial.println("[OK]   NEO-6M GPS");
  } else {
    Serial.println("[WARN] GPS not responding — GPS fields will be NAN");
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);  delay(10);
  digitalWrite(LORA_RST, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println("[ERROR] LoRa RFM95W init failed! Check wiring. Halting.");
    while (1) delay(100);
  }
  rf95.setFrequency(LORA_FREQ);
  rf95.setTxPower(13, false);
  mesh.init();

  Serial.printf("[MESH] Node %d ready | ACCEL:%s BMP:%s GPS:%s\n\n",
    MY_ADDRESS,
    accelOK ? "OK" : "FAIL",
    bmpOK   ? "OK" : "FAIL",
    gpsOK   ? "OK" : "FAIL");
}

void loop() {
  feedGPS(200);

  SensorPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.nodeId      = MY_ADDRESS;
  pkt.hopCount    = 0;
  pkt.sensorFlags = (accelOK ? 0x01 : 0x00)
                  | (bmpOK   ? 0x02 : 0x00)
                  | (gpsOK   ? 0x04 : 0x00);

  if (accelOK) {
    sensors_event_t evt;
    accel.getEvent(&evt);
    pkt.accelX = evt.acceleration.x;
    pkt.accelY = evt.acceleration.y;
    pkt.accelZ = evt.acceleration.z;
  } else {
    pkt.accelX = pkt.accelY = pkt.accelZ = NAN;
  }

  if (bmpOK) {
    bmp5_sensor_data d = {0, 0};
    if (bmp.getSensorData(&d) == BMP5_OK) {
      pkt.pressure    = d.pressure / 100.0f;
      pkt.temperature = d.temperature;
    } else {
      pkt.pressure = pkt.temperature = NAN;
    }
  } else {
    pkt.pressure = pkt.temperature = NAN;
  }

  feedGPS(200);
  if (gpsOK && gps.location.isValid()) {
    pkt.latitude  = (float)gps.location.lat();
    pkt.longitude = (float)gps.location.lng();
    pkt.altitude  = (float)gps.altitude.meters();
  } else {
    pkt.latitude = pkt.longitude = pkt.altitude = NAN;
  }

  uint8_t buf[sizeof(SensorPacket)];
  memcpy(buf, &pkt, sizeof(SensorPacket));

  uint8_t err = mesh.sendtoWait(buf, sizeof(SensorPacket), PARENT_ADDRESS);

  if (err == RH_ROUTER_ERROR_NONE) {
    Serial.printf("[TX OK ] Node%d->Parent | A:(%.2f,%.2f,%.2f) "
                  "P:%.1fhPa T:%.1fC GPS:(%.5f,%.5f,%.1fm) Flags:0x%02X\n",
      pkt.nodeId, pkt.accelX, pkt.accelY, pkt.accelZ,
      pkt.pressure, pkt.temperature,
      pkt.latitude, pkt.longitude, pkt.altitude,
      pkt.sensorFlags);
  } else {
    Serial.printf("[TX ERR] Node%d send failed (code %d)\n", MY_ADDRESS, err);
  }

  delay(1000);
}