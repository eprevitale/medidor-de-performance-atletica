/*
 * MPA
 * Core: "esp32 by Espressif" 3.x | Placa: ESP32S3 Dev Module
 *
 * Serviço 1 (Controle): ao receber o comando "start" inicia o streaming de dados.
 * Serviço 2 (Data): stream de amostras de IMU por notify.
 */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define UUID_SUFFIX     "-8f3a-4c1e-9b7a-0a1b2c3d4e5f"
#define SVC_PROV_UUID   "6d500001" UUID_SUFFIX
#define CHR_CMD_UUID    "6d500002" UUID_SUFFIX   // "start" | "stop"
#define CHR_STATUS_UUID "6d500003" UUID_SUFFIX   // read + notify
#define SVC_DATA_UUID   "6d500010" UUID_SUFFIX
#define CHR_IMU_UUID    "6d500011" UUID_SUFFIX   // notify

#define SAMPLE_HZ 100   // taxa do stream fake -> mudar para 400

// Define a estrutura de dados do IMU
// 18 bytes: cabe no MTU padrão (23 -> 20 bytes úteis, 3 bytes de overhead). Depois dá pra agrupar várias amostras por pacote.
struct __attribute__((packed)) ImuSample {
  uint32_t t_ms;
  uint16_t seq;
  int16_t ax, ay, az, gx, gy, gz;
};

// Declara variáveiis do BLE como ponteiros
BLEServer* server;
BLECharacteristic *chCmd, *chStatus, *chImu;

// Declara variáveis de controle
volatile bool connected = false;
volatile bool streaming = false;
uint16_t seq = 0;
uint32_t lastSampleUs = 0;

// Define o status do BLE
void setStatus(const String& s) {
  Serial.println("[status] " + s);
  chStatus->setValue((uint8_t*)s.c_str(), s.length());
  if (connected) chStatus->notify();
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
    Serial.println("[ble] cliente desconectado, voltando a anunciar");
    BLEDevice::startAdvertising();
    // iniciar salvamento de dados local
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
      if (val == "start") { streaming = true; seq = 0; }
      else if (val == "stop") streaming = false;
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

  // Serviço de provisionamento (comandos e status)
  BLEService* prov = server->createService(SVC_PROV_UUID);
  chCmd    = prov->createCharacteristic(CHR_CMD_UUID,    BLECharacteristic::PROPERTY_WRITE);
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
      s.seq = seq++;
      s.ax = (int16_t)(1000 * sinf(2 * PI * 1.0f * t));
      s.ay = (int16_t)(1000 * cosf(2 * PI * 1.0f * t));
      s.az = 4096;  // ~1 g
      s.gx = s.gy = s.gz = 0;
      chImu->setValue((uint8_t*)&s, sizeof(s));
      chImu->notify();
    }
  }
}
