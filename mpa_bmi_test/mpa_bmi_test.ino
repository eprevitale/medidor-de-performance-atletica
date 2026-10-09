/*
 * MPA - Streaming de IMU (BMI160) via BLE, com agrupamento (batch) de amostras por pacote
 * Core: "esp32 by Espressif" 3.x | Placa: ESP32S3 Dev Module
 *
 * Serviço de controle (UUID final 0010): comandos "start"/"stop" + status (read/notify).
 * Serviço de dados (UUID final 0020): stream de IMU (BMI160 real), agrupado em lotes de BATCH_SIZE amostras.
 *
 * Escala dos valores brutos (int16) enviados:
 *   Acelerômetro ±8 g     -> 4096 LSB/g
 *   Giroscópio  ±2000 °/s -> 16.4 LSB/(°/s)
 */
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>   // CCCD: necessário nas características com NOTIFY (core 3.0.x, Bluedroid)
#include <Adafruit_NeoPixel.h>

// BLE UUIDs
#define UUID_SUFFIX     "-8f3a-4c1e-9b7a-0a1b2c3d4e5f"
#define SVC_PROV_UUID   "6d500010" UUID_SUFFIX
#define CHR_CMD_UUID    "6d500011" UUID_SUFFIX   // "start" | "stop"
#define CHR_STATUS_UUID "6d500012" UUID_SUFFIX   // read + notify
#define SVC_DATA_UUID   "6d500020" UUID_SUFFIX
#define CHR_IMU_UUID    "6d500021" UUID_SUFFIX

// Data
#define BATCH_SIZE  9     // amostras por pacote BLE (ODR do sensor = 400 Hz, ver bmiInit)

// I2C (ajuste para os seus pinos)
#define I2C_SDA 8
#define I2C_SCL 9

// RGB LED
#define PIN_RGB 48
#define NUM_LEDS 1

// Registradores do BMI160
#define BMI_REG_CHIP_ID   0x00
#define BMI_REG_ERR       0x02
#define BMI_REG_PMU_STAT  0x03
#define BMI_REG_DATA      0x0C   // gyro X..Z (6 bytes) seguido de acc X..Z (6 bytes)
#define BMI_REG_STATUS    0x1B   // bit7 drdy_acc, bit6 drdy_gyr
#define BMI_REG_ACC_CONF  0x40
#define BMI_REG_ACC_RANGE 0x41
#define BMI_REG_GYR_CONF  0x42
#define BMI_REG_GYR_RANGE 0x43
#define BMI_REG_CMD       0x7E

// 18 bytes por amostra
struct __attribute__((packed)) ImuSample {
  uint32_t t_ms;
  uint16_t seq;
  int16_t ax, ay, az, gx, gy, gz;
};

// Agrupa várias ImuSample em um único pacote BLE
// header de 3 bytes + até 9*18 = 162 bytes -> 165 bytes (precisa de MTU >= 168)
struct __attribute__((packed)) ImuBatch {
  uint16_t  batch_seq;
  uint8_t   count;
  ImuSample samples[BATCH_SIZE];
};

BLEServer* server;
BLECharacteristic *chCmd, *chStatus, *chImu;

volatile bool connected = false;
volatile bool streaming = false;

uint16_t sampleSeq = 0;
uint16_t batchSeq  = 0;

ImuBatch batch;
uint8_t  batchFill = 0;

uint8_t bmiAddr = 0x68;
bool    imuOk   = false;
uint8_t i2cFails = 0;

Adafruit_NeoPixel strip(NUM_LEDS, PIN_RGB, NEO_GRB + NEO_KHZ800);

// ---------------------------------------------------------------- BMI160

bool bmiWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(bmiAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool bmiReadBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(bmiAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(bmiAddr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Procura o sensor em 0x68 (SDO em GND) e 0x69 (SDO em 3V3)
bool bmiProbe() {
  const uint8_t addrs[] = {0x68, 0x69};
  for (uint8_t a : addrs) {
    bmiAddr = a;
    uint8_t id = 0;
    if (bmiReadBytes(BMI_REG_CHIP_ID, &id, 1) && id == 0xD1) return true;
  }
  return false;
}

bool bmiInit() {
  if (!bmiProbe()) {
    Serial.println("[imu] BMI160 nao encontrado (CHIP_ID != 0xD1)");
    return false;
  }
  Serial.printf("[imu] BMI160 encontrado em 0x%02X\n", bmiAddr);

  bmiWrite(BMI_REG_CMD, 0xB6);   // soft reset
  delay(100);

  bmiWrite(BMI_REG_CMD, 0x11);   // acelerômetro -> modo normal
  delay(10);
  bmiWrite(BMI_REG_CMD, 0x15);   // giroscópio  -> modo normal
  delay(100);

  uint8_t pmu = 0, err = 0;
  for (int i = 0; i < 5; i++) {
    bmiReadBytes(BMI_REG_PMU_STAT, &pmu, 1);
    if (pmu == 0x14) break;
    delay(20);
  }
  bmiReadBytes(BMI_REG_ERR, &err, 1);
  Serial.printf("[imu] PMU_STATUS=0x%02X (esperado 0x14) ERR_REG=0x%02X\n", pmu, err);
  if (pmu != 0x14) return false;

  bmiWrite(BMI_REG_ACC_CONF,  0x2A);  // 400 Hz, filtro normal
  bmiWrite(BMI_REG_ACC_RANGE, 0x08);  // ±8 g
  bmiWrite(BMI_REG_GYR_CONF,  0x2A);  // 400 Hz, filtro normal
  bmiWrite(BMI_REG_GYR_RANGE, 0x00);  // ±2000 °/s
  delay(10);
  return true;
}

// Retorna true se havia amostra nova (acc e gyro prontos) e preencheu 's'
bool bmiReadSample(ImuSample& s) {
  uint8_t st = 0;
  if (!bmiReadBytes(BMI_REG_STATUS, &st, 1)) { i2cFails++; return false; }
  if ((st & 0xC0) != 0xC0) return false;   // ainda sem dado novo

  uint8_t b[12];
  if (!bmiReadBytes(BMI_REG_DATA, b, 12)) { i2cFails++; return false; }
  i2cFails = 0;

  auto i16 = [&](int k) { return (int16_t)((b[k + 1] << 8) | b[k]); };
  s.gx = i16(0);  s.gy = i16(2);  s.gz = i16(4);
  s.ax = i16(6);  s.ay = i16(8);  s.az = i16(10);
  return true;
}

// ---------------------------------------------------------------- BLE

void setStatus(const String& s) {
  Serial.println("[status] " + s);
  chStatus->setValue((uint8_t*)s.c_str(), s.length());
  if (connected) chStatus->notify();
}

void flushBatch() {
  if (batchFill == 0) return;
  batch.batch_seq = batchSeq++;
  batch.count = batchFill;
  size_t bytes = sizeof(batch.batch_seq) + sizeof(batch.count) + batchFill * sizeof(ImuSample);
  chImu->setValue((uint8_t*)&batch, bytes);
  chImu->notify();
  batchFill = 0;
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    connected = true;
    Serial.println("[ble] cliente conectado");
  }
  void onDisconnect(BLEServer*) override {
    connected = false;
    streaming = false;
    batchFill = 0;
    Serial.println("[ble] cliente desconectado, voltando a anunciar");
    BLEDevice::startAdvertising();
    // TODO adicionar lógica de salvamento de dados local
  }
};

class WriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto raw = c->getValue();
    String val(raw.c_str());
    if (c == chCmd) {
      val.trim();
      Serial.println("[cmd] " + val);
      if (val == "start") {
        if (!imuOk) {
          setStatus("imu_error");
          return;
        }
        sampleSeq = 0;
        batchSeq  = 0;
        batchFill = 0;
        i2cFails  = 0;
        streaming = true;
      } else if (val == "stop") {
        streaming = false;
        flushBatch();   // manda o que sobrou, mesmo incompleto
      }
    }
  }
};

// ---------------------------------------------------------------- LED (sem delay)

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  static uint32_t last = 0xFFFFFFFF;
  uint32_t c = strip.Color(r, g, b);
  if (c == last) return;     // evita strip.show() desnecessário
  last = c;
  strip.setPixelColor(0, c);
  strip.show();
}

void updateLed() {
  static uint32_t lastToggle = 0;
  static bool on = false;
  uint32_t now = millis();

  if (!connected) {
    if (now - lastToggle >= 250) { lastToggle = now; on = !on; setLed(0, 0, on ? 255 : 0); }   // azul piscando
  } else if (streaming) {
    if (now - lastToggle >= 250) { lastToggle = now; on = !on; setLed(on ? 255 : 0, 0, 0); }   // vermelho piscando
  } else {
    setLed(0, 255, 0);                                                                         // verde fixo
  }
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  Serial.begin(115200);
  delay(1500);
  strip.begin();
  strip.show();

  // IMU: tenta 400 kHz e, se falhar, 100 kHz
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(20);
  Wire.setClock(400000);
  imuOk = bmiInit();
  if (!imuOk) {
    Serial.println("[imu] falhou a 400 kHz, tentando 100 kHz");
    Wire.setClock(100000);
    imuOk = bmiInit();
  }
  Serial.println(imuOk ? "[imu] OK" : "[imu] FALHA");

  char name[16];
  snprintf(name, sizeof(name), "MPA-%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  Serial.printf("Iniciando BLE como %s\n", name);

  BLEDevice::init(name);
  BLEDevice::setMTU(247);

  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  WriteCB* wcb = new WriteCB();

  BLEService* prov = server->createService(SVC_PROV_UUID);
  chCmd    = prov->createCharacteristic(CHR_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  chStatus = prov->createCharacteristic(CHR_STATUS_UUID,
               BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  chCmd->setCallbacks(wcb);
  chStatus->addDescriptor(new BLE2902());
  prov->start();

  BLEService* data = server->createService(SVC_DATA_UUID);
  chImu = data->createCharacteristic(CHR_IMU_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  chImu->addDescriptor(new BLE2902());
  data->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_PROV_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  setStatus(imuOk ? "idle" : "imu_error");
}

void loop() {
  updateLed();

  if (streaming && connected) {
    ImuSample s;
    if (bmiReadSample(s)) {
      s.t_ms = millis();
      s.seq  = sampleSeq++;
      batch.samples[batchFill++] = s;
      if (batchFill == BATCH_SIZE) flushBatch();
    }

    // Muitas falhas seguidas de I2C: para o stream e avisa o app
    if (i2cFails > 50) {
      streaming = false;
      flushBatch();
      setStatus("imu_error");
    }
  }
}
