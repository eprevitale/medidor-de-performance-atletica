/*
 * MPA - Streaming de IMU via BLE, com agrupamento (batch) de amostras por pacote
 * Core: "esp32 by Espressif" 3.x | Placa: ESP32S3 Dev Module
 *
 * Serviço de controle (UUID final 0010): comandos "start"/"stop" + status (read/notify).
 * Serviço de dados (UUID final 0020): stream de IMU FAKE, agrupado em lotes de BATCH_SIZE amostras.
 */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define UUID_SUFFIX     "-8f3a-4c1e-9b7a-0a1b2c3d4e5f"
#define SVC_PROV_UUID   "6d500010" UUID_SUFFIX
#define CHR_CMD_UUID    "6d500011" UUID_SUFFIX   // "start" | "stop"
#define CHR_STATUS_UUID "6d500012" UUID_SUFFIX   // read + notify
#define SVC_DATA_UUID   "6d500020" UUID_SUFFIX
#define CHR_IMU_UUID    "6d500021" UUID_SUFFIX

#define SAMPLE_HZ   400
#define BATCH_SIZE  9     // amostras por pacote BLE

// Define a estrutura de dados do IMU
// 18 bytes: cabe no MTU padrão (23 -> 20 bytes úteis, 3 bytes de overhead).
struct __attribute__((packed)) ImuSample {
  uint32_t t_ms;
  uint16_t seq;
  int16_t ax, ay, az, gx, gy, gz;
};  // 18 bytes

// Agrupa várias ImuSample em um único pacote BLE
struct __attribute__((packed)) ImuBatch {
  uint16_t  batch_seq;          // contador do pacote
  uint8_t   count;              // quantas amostras estão preenchidas neste pacote
  ImuSample samples[BATCH_SIZE];
};  // header de 3 bytes + até 9*18 = 162 bytes de amostras

// Declara variáveiis do BLE como ponteiros
BLEServer* server;
BLECharacteristic *chCmd, *chStatus, *chImu;

// Declara variáveis de controle
volatile bool connected = false;
volatile bool streaming = false;

uint16_t sampleSeq = 0;   // contador global de amostras
uint16_t batchSeq  = 0;   // contador de pacotes (para detectar pacotes perdidos)
uint32_t lastSampleUs = 0;

ImuBatch batch;
uint8_t  batchFill = 0;   // quantas amostras já foram colocadas no pacote atual

// Define o status do BLE
void setStatus(const String& s) {
  Serial.println("[status] " + s);
  chStatus->setValue((uint8_t*)s.c_str(), s.length());
  if (connected) chStatus->notify();
}

// Envia o pacote atual (mesmo que incompleto) e zera o contador de preenchimento
void flushBatch() {
  if (batchFill == 0) return; // nada a enviar
  batch.batch_seq = batchSeq++;
  batch.count = batchFill;
  size_t bytes = sizeof(batch.batch_seq) + sizeof(batch.count) + batchFill * sizeof(ImuSample);
  chImu->setValue((uint8_t*)&batch, bytes);
  chImu->notify();
  batchFill = 0;
}

// Configura callbacks de conexão e desconexão, herdando da classe padrão do BLE
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

// Configura callback de escrita nas características do BLE
class WriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto raw = c->getValue();
    String val(raw.c_str());
    if (c == chCmd) {
      val.trim();
      Serial.println("[cmd] " + val);
      if (val == "start") {
        streaming = true;
        sampleSeq = 0;
        batchSeq  = 0;
        batchFill = 0;
      } else if (val == "stop") {
        streaming = false;
        flushBatch();   // manda o que sobrou, mesmo incompleto
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Define o nome do dispositivo ao ser descoberto
  char name[16];
  snprintf(name, sizeof(name), "MPA-%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  Serial.printf("Iniciando BLE como %s\n", name);

  BLEDevice::init(name);
  BLEDevice::setMTU(247); // Define a unidade máxima de transmissão como 247 bytes

  server = BLEDevice::createServer();    // inicia o servidor
  server->setCallbacks(new ServerCB());  // aplica os callbacks
  WriteCB* wcb = new WriteCB();

  // Serviço de controle (comandos e status)
  BLEService* prov = server->createService(SVC_PROV_UUID);
  chCmd    = prov->createCharacteristic(CHR_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  chStatus = prov->createCharacteristic(CHR_STATUS_UUID,
               BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  chCmd->setCallbacks(wcb);
  prov->start();

  // Serviço de dados (transmissão de dados em si)
  BLEService* data = server->createService(SVC_DATA_UUID);
  chImu = data->createCharacteristic(CHR_IMU_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  data->start();

  // Inicia o modo de descoberta
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_PROV_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  setStatus("idle");
}

void loop() {
  // Stream fake de IMU
  if (streaming && connected) {
    uint32_t now = micros();
    if (now - lastSampleUs >= 1000000UL / SAMPLE_HZ) {
      lastSampleUs = now;
      float t = millis() / 1000.0f;

      ImuSample s;
      s.t_ms = millis();
      s.seq  = sampleSeq++;
      s.ax = (int16_t)(1000 * sinf(2 * PI * 1.0f * t));
      s.ay = (int16_t)(1000 * cosf(2 * PI * 1.0f * t));
      s.az = 4096;  // ~1 g
      s.gx = s.gy = s.gz = 0;

      batch.samples[batchFill++] = s;
      if (batchFill == BATCH_SIZE) flushBatch();
    }
  }
}
