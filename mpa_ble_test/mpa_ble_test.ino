/*
 * MPA - Teste rápido de BLE no ESP32-S3
 * Core: "esp32 by Espressif" 3.x | Placa: ESP32S3 Dev Module
 *
 * Serviço 1 (Provisioning): recebe SSID / senha / tenant_id por escrita BLE,
 *   e ao receber o comando "connect" tenta conectar no WiFi (testa BLE + WiFi juntos).
 * Serviço 2 (Data): stream de amostras de IMU FAKE por notify (testa vazão/perda de pacotes).
 *
 * Testar com o app nRF Connect (Android/iOS), escrevendo como UTF-8 / texto.
 */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <WiFi.h>

#define UUID_SUFFIX     "-8f3a-4c1e-9b7a-0a1b2c3d4e5f"
#define SVC_PROV_UUID   "6d500001" UUID_SUFFIX
#define CHR_SSID_UUID   "6d500002" UUID_SUFFIX
#define CHR_PASS_UUID   "6d500003" UUID_SUFFIX
#define CHR_TENANT_UUID "6d500004" UUID_SUFFIX
#define CHR_CMD_UUID    "6d500005" UUID_SUFFIX   // "connect" | "start" | "stop"
#define CHR_STATUS_UUID "6d500006" UUID_SUFFIX   // read + notify
#define SVC_DATA_UUID   "6d500010" UUID_SUFFIX
#define CHR_IMU_UUID    "6d500011" UUID_SUFFIX   // notify

#define SAMPLE_HZ 100   // taxa do stream fake

// 18 bytes: cabe no MTU padrão (23 -> 20 bytes úteis). Depois dá pra agrupar várias amostras por pacote.
struct __attribute__((packed)) ImuSample {
  uint32_t t_ms;
  uint16_t seq;
  int16_t ax, ay, az, gx, gy, gz;
};

BLEServer* server;
BLECharacteristic *chSsid, *chPass, *chTenant, *chCmd, *chStatus, *chImu;

String ssid, pass, tenant;
volatile bool connected = false;
volatile bool wantWifi = false;
volatile bool streaming = false;
uint16_t seq = 0;
uint32_t lastSampleUs = 0;

void setStatus(const String& s) {
  Serial.println("[status] " + s);
  chStatus->setValue((uint8_t*)s.c_str(), s.length());
  if (connected) chStatus->notify();
}

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
  }
};

class WriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto raw = c->getValue();          // std::string (core 2.x) ou String (core 3.x)
    String val(raw.c_str());
    if (c == chSsid) {
      ssid = val;
      Serial.println("[prov] ssid = " + ssid);
    } else if (c == chPass) {
      pass = val;
      Serial.printf("[prov] senha recebida (%d chars)\n", pass.length());
    } else if (c == chTenant) {
      tenant = val;
      Serial.println("[prov] tenant_id = " + tenant);
    } else if (c == chCmd) {
      val.trim();
      Serial.println("[cmd] " + val);
      if (val == "connect") wantWifi = true;
      else if (val == "start") { streaming = true; seq = 0; }
      else if (val == "stop") streaming = false;
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1500);

  char name[16];
  snprintf(name, sizeof(name), "MPA-%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  Serial.printf("Iniciando BLE como %s\n", name);

  BLEDevice::init(name);
  BLEDevice::setMTU(247);

  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  WriteCB* wcb = new WriteCB();

  // --- Serviço de provisionamento ---
  BLEService* prov = server->createService(SVC_PROV_UUID);
  chSsid   = prov->createCharacteristic(CHR_SSID_UUID,   BLECharacteristic::PROPERTY_WRITE);
  chPass   = prov->createCharacteristic(CHR_PASS_UUID,   BLECharacteristic::PROPERTY_WRITE);
  chTenant = prov->createCharacteristic(CHR_TENANT_UUID, BLECharacteristic::PROPERTY_WRITE);
  chCmd    = prov->createCharacteristic(CHR_CMD_UUID,    BLECharacteristic::PROPERTY_WRITE);
  chStatus = prov->createCharacteristic(CHR_STATUS_UUID,
               BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  chSsid->setCallbacks(wcb);
  chPass->setCallbacks(wcb);
  chTenant->setCallbacks(wcb);
  chCmd->setCallbacks(wcb);
  prov->start();

  // --- Serviço de dados ---
  BLEService* data = server->createService(SVC_DATA_UUID);
  chImu = data->createCharacteristic(CHR_IMU_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  data->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_PROV_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  setStatus("idle");
}

void loop() {
  // Conexão WiFi fora do callback BLE (é bloqueante)
  if (wantWifi) {
    wantWifi = false;
    if (ssid.isEmpty()) {
      setStatus("erro:sem_ssid");
    } else {
      setStatus("wifi_connecting");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid.c_str(), pass.c_str());
      uint32_t t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
      if (WiFi.status() == WL_CONNECTED) setStatus("wifi_ok:" + WiFi.localIP().toString());
      else {
        setStatus("wifi_fail:" + String((int)WiFi.status()));
        int n = WiFi.scanNetworks();
        for (int i = 0; i < n; i++)
          Serial.printf("  %s | canal %d | %d dBm | auth %d\n",
            WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i));
      }
    }
  }

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
