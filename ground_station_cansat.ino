// ============================================================================
//  GROUND STATION — BSAET CanSat 2026
//  Flash to a dedicated ESP32-S3 + RFM95W connected to laptop via USB.
//  Open Serial Monitor at 115200 baud to see all incoming data.
//
//  NO sensors needed — LoRa radio only.
//
//  Libraries required:
//    - RadioHead by Mike McCauley
// ============================================================================

#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>
#include <math.h>

#define MY_ADDRESS   20
#define MAX_CHILDREN  5

#define LORA_SCK  12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_CS   10
#define LORA_DIO0 41
#define LORA_RST  40
#define LORA_FREQ 915.0

RH_RF95 rf95(LORA_CS, LORA_DIO0);
RHMesh  mesh(rf95, MY_ADDRESS);

#pragma pack(1)
struct SensorPacket {
  uint8_t nodeId;
  uint8_t hopCount;
  float accelX, accelY, accelZ;
  float pressure;
  float temperature;
  float latitude;
  float longitude;
  float altitude;
  uint8_t sensorFlags;
};

struct DownlinkBundle {
  uint8_t packetType;
  SensorPacket parent;
  uint8_t childCount;
  SensorPacket children[MAX_CHILDREN];
};
#pragma pack()

void printNode(const char* label, const SensorPacket& s) {
  Serial.printf("  +-- %s (Node %d | Hops: %d | Flags: 0x%02X)\n",
    label, s.nodeId, s.hopCount, s.sensorFlags);
  if (!isnan(s.accelX))
    Serial.printf("  |   Accel:    X=%+.3f  Y=%+.3f  Z=%+.3f m/s2\n",
      s.accelX, s.accelY, s.accelZ);
  else
    Serial.println("  |   Accel:    [NO DATA]");
  if (!isnan(s.pressure))
    Serial.printf("  |   Pressure: %.2f hPa   Temp: %.2f C\n",
      s.pressure, s.temperature);
  else
    Serial.println("  |   Pressure: [NO DATA]");
  if (!isnan(s.latitude))
    Serial.printf("  |   GPS:      lat=%.6f  lon=%.6f  alt=%.1fm\n",
      s.latitude, s.longitude, s.altitude);
  else
    Serial.println("  |   GPS:      [NO FIX]");
  Serial.println("  |");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n==========================================");
  Serial.println("  BSAET CanSat 2026 - Ground Station");
  Serial.println("==========================================\n");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);  delay(10);
  digitalWrite(LORA_RST, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println("[ERROR] LoRa RFM95W init failed! Check wiring. Halting.");
    while (1) delay(100);
  }
  rf95.setFrequency(LORA_FREQ);
  rf95.setTxPower(20, false);
  mesh.init();

  Serial.println("[GS] Ready - waiting for downlink...\n");
}

void loop() {
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (mesh.recvfromAck(buf, &len, &from)) {
    if (len < 1) return;
    uint8_t packetType = buf[0];

    if (packetType == 0xAA && len == sizeof(DownlinkBundle)) {
      DownlinkBundle bundle;
      memcpy(&bundle, buf, sizeof(DownlinkBundle));

      Serial.println("==========================================");
      Serial.printf("  DOWNLINK  |  T+%lums  |  RSSI: %ddBm\n",
        millis(), rf95.lastRssi());
      Serial.printf("  Children reporting: %d / 5\n", bundle.childCount);
      Serial.println("==========================================");

      printNode("PARENT", bundle.parent);

      if (bundle.childCount == 0) {
        Serial.println("  (No child data received yet)");
      } else {
        for (uint8_t i = 0; i < bundle.childCount; i++) {
          char label[16];
          snprintf(label, sizeof(label), "CHILD %d", bundle.children[i].nodeId);
          printNode(label, bundle.children[i]);
        }
      }
      Serial.println("==========================================\n");

    } else {
      Serial.printf("[GS] Unknown packet from node %d (type=0x%02X len=%d)\n",
        from, packetType, len);
    }
  }
}