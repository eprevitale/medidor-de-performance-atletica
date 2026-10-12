/*
 * MPA - IMU (BMI160) via BLE: modo ao vivo + modo mapeamento (grava no aparelho)
 * Core: "esp32 by Espressif" 3.x | Placa: ESP32S3 Dev Module
 *
 * IMPORTANTE: Tools > Partition Scheme precisa ter area SPIFFS/LittleFS
 *             (ex.: "Default 4MB with spiffs" ou "8M with spiffs").
 *             Se a placa tiver PSRAM, ative Tools > PSRAM para gravar series mais longas.
 *
 * Servico de controle (UUID 0010): comandos de texto (write) + status (read/notify)
 *   Modo ao vivo:   "start" | "stop"
 *   Mapeamento:     "rec|<atleta>|<exercicio>|<carga_kg>"
 *                   "recstop"                   para e salva em arquivo (LittleFS)
 *                   "list"                      lista os arquivos salvos
 *                   "get|<arquivo>"             envia o arquivo em blocos (caracteristica 0022)
 *                   "del|<arquivo>"             apaga o arquivo
 *
 * Servico de dados (UUID 0020): 0021 = stream ao vivo (batch), 0022 = blocos de arquivo
 *
 * Formato do arquivo (.bin): FileHeader (68 bytes) + amostras de 12 bytes
 *   (gx, gy, gz, ax, ay, az em int16 little-endian, valores BRUTOS, 400 Hz).
 *   O offset do giroscopio medido na calibracao fica no cabecalho (nao e subtraido).
 *
 * Escala dos valores brutos: acelerometro +-8 g = 4096 LSB/g; giroscopio +-2000 dps = 16.4 LSB/dps
 *
 * VERSAO 2 do arquivo: inclui carga (kg) no cabecalho.
 *   version=1 → header de 64 bytes (compativel com arquivos antigos)
 *   version=2 → header de 68 bytes (com carga)
 *
 * TRANSFERENCIA DE ARQUIVO (Correcao 3 - pacote bem menor):
 *   XFER_CHUNK = 40 (pacote de 44 bytes cabe em qualquer MTU BLE)
 *   XFER_DELAY_MS = 15 (leve pausa entre pacotes)
 */
#include <Wire.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>

// BLE UUIDs
#define UUID_SUFFIX     "-8f3a-4c1e-9b7a-0a1b2c3d4e5f"
#define SVC_PROV_UUID   "6d500010" UUID_SUFFIX
#define CHR_CMD_UUID    "6d500011" UUID_SUFFIX   // comandos de texto
#define CHR_STATUS_UUID "6d500012" UUID_SUFFIX   // read + notify
#define SVC_DATA_UUID   "6d500020" UUID_SUFFIX
#define CHR_IMU_UUID    "6d500021" UUID_SUFFIX   // stream ao vivo
#define CHR_XFER_UUID   "6d500022" UUID_SUFFIX   // blocos de arquivo (notify)

// Modo ao vivo
#define BATCH_SIZE        9      // amostras por pacote BLE

// Modo mapeamento
#define REC_RATE_HZ       400
#define REC_SAMPLE_BYTES  12
#define COUNTDOWN_MS      5000

// Transferencia de arquivo (Correcao 3)
#define XFER_CHUNK        40     // bytes de dados por pacote (+4 de offset) -> 44 bytes
#define XFER_DELAY_MS     15     // pausa entre pacotes

// I2C
#define I2C_SDA 8
#define I2C_SCL 9

// RGB LED
#define PIN_RGB 48
#define NUM_LEDS 1

// Registradores do BMI160
#define BMI_REG_CHIP_ID   0x00
#define BMI_REG_ERR       0x02
#define BMI_REG_PMU_STAT  0x03
#define BMI_REG_DATA      0x0C
#define BMI_REG_STATUS    0x1B
#define BMI_REG_ACC_CONF  0x40
#define BMI_REG_ACC_RANGE 0x41
#define BMI_REG_GYR_CONF  0x42
#define BMI_REG_GYR_RANGE 0x43
#define BMI_REG_CMD       0x7E

// 18 bytes por amostra (modo ao vivo)
struct __attribute__((packed)) ImuSample {
  uint32_t t_ms;
  uint16_t seq;
  int16_t ax, ay, az, gx, gy, gz;
};

struct __attribute__((packed)) ImuBatch {
  uint16_t  batch_seq;
  uint8_t   count;
  ImuSample samples[BATCH_SIZE];
};

// Cabecalho do arquivo de sessao (68 bytes, version = 2)
struct __attribute__((packed)) FileHeader {
  char     magic[4];        // "MPA1"
  uint8_t  version;         // 2
  uint8_t  sampleBytes;     // 12
  uint16_t rateHz;          // 400
  uint32_t sampleCount;
  uint32_t missed;
  uint32_t crc32;
  uint32_t bootMs;
  int16_t  gyroOff[3];
  uint8_t  accRange;        // 0x08 = +-8 g
  uint8_t  gyrRange;        // 0x00 = +-2000 dps
  char     athlete[20];
  char     exercise[12];
  uint16_t loadKg;          // NOVO (version 2)
  uint16_t _pad;            // padding
};
static_assert(sizeof(FileHeader) == 68, "FileHeader deve ter 68 bytes");

BLEServer* server;
BLECharacteristic *chCmd, *chStatus, *chImu, *chXfer;

volatile bool connected = false;
volatile bool streaming = false;

uint16_t sampleSeq = 0;
uint16_t batchSeq  = 0;
ImuBatch batch;
uint8_t  batchFill = 0;

uint8_t  bmiAddr = 0x68;
bool     imuOk   = false;
uint8_t  i2cFails = 0;
uint32_t lastSensorTime = 0;

volatile bool startRequested = false;
int16_t gyroOff[3] = {0, 0, 0};

bool fsOk = false;

// Mapeamento
enum RecState : uint8_t { REC_IDLE, REC_CALIB, REC_COUNTDOWN, REC_RECORDING, REC_SAVING };
volatile RecState recState = REC_IDLE;
volatile bool recCmdStart = false, recCmdStop = false;
volatile bool listRequested = false, getRequested = false, delRequested = false;
String cmdAthlete, cmdExercise;
uint16_t cmdLoadKg = 0;
String pendingGet, pendingDel;

char     recAthlete[20], recExercise[12];
uint16_t recLoadKg = 0;
uint8_t* recBuf = nullptr;
size_t   recCap = 0, recLen = 0;
uint32_t recMissed = 0, recBootMs = 0, recT0 = 0, prevSensorTime = 0;
bool     haveLast = false;

Adafruit_NeoPixel strip(NUM_LEDS, PIN_RGB, NEO_GRB + NEO_KHZ800);

void setLed(uint8_t r, uint8_t g, uint8_t b);

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

  bmiWrite(BMI_REG_CMD, 0xB6);
  delay(100);

  bmiWrite(BMI_REG_CMD, 0x11);
  delay(10);
  bmiWrite(BMI_REG_CMD, 0x15);
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

  bmiWrite(BMI_REG_ACC_CONF,  0x2A);
  bmiWrite(BMI_REG_ACC_RANGE, 0x08);
  bmiWrite(BMI_REG_GYR_CONF,  0x2A);
  bmiWrite(BMI_REG_GYR_RANGE, 0x00);
  delay(10);
  return true;
}

bool bmiReadSample(ImuSample& s) {
  uint8_t st = 0;
  if (!bmiReadBytes(BMI_REG_STATUS, &st, 1)) { i2cFails++; return false; }
  if ((st & 0xC0) != 0xC0) return false;

  uint8_t b[15];
  if (!bmiReadBytes(BMI_REG_DATA, b, 15)) { i2cFails++; return false; }
  i2cFails = 0;

  auto i16 = [&](int k) { return (int16_t)((b[k + 1] << 8) | b[k]); };
  s.gx = i16(0);  s.gy = i16(2);  s.gz = i16(4);
  s.ax = i16(6);  s.ay = i16(8);  s.az = i16(10);
  lastSensorTime = (uint32_t)b[12] | ((uint32_t)b[13] << 8) | ((uint32_t)b[14] << 16);
  return true;
}

bool calibrateGyro() {
  const uint16_t N = 400;
  const int16_t MAX_P2P = 50;
  int32_t sum[3] = {0, 0, 0};
  int16_t mn[3] = {32767, 32767, 32767};
  int16_t mx[3] = {-32768, -32768, -32768};
  uint16_t n = 0;
  uint32_t t0 = millis();

  while (n < N && millis() - t0 < 3000) {
    ImuSample s;
    if (!bmiReadSample(s)) continue;
    int16_t g[3] = {s.gx, s.gy, s.gz};
    for (int i = 0; i < 3; i++) {
      sum[i] += g[i];
      if (g[i] < mn[i]) mn[i] = g[i];
      if (g[i] > mx[i]) mx[i] = g[i];
    }
    n++;
  }
  if (n < N) {
    Serial.println("[calib] amostras insuficientes");
    return false;
  }
  for (int i = 0; i < 3; i++) {
    if (mx[i] - mn[i] > MAX_P2P) {
      Serial.printf("[calib] movimento no eixo %d (pico a pico = %d LSB)\n", i, mx[i] - mn[i]);
      return false;
    }
  }
  for (int i = 0; i < 3; i++) gyroOff[i] = (int16_t)(sum[i] / N);
  Serial.printf("[calib] offset do giroscopio (LSB): %d %d %d\n", gyroOff[0], gyroOff[1], gyroOff[2]);
  return true;
}

// ---------------------------------------------------------------- Utilitarios

uint32_t crc32_calc(const uint8_t* d, size_t n) {
  uint32_t c = 0xFFFFFFFFUL;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c & 1) ? ((c >> 1) ^ 0xEDB88320UL) : (c >> 1);
  }
  return c ^ 0xFFFFFFFFUL;
}

String sanitizeName(String s, size_t maxLen) {
  s.trim();
  s.toLowerCase();
  String out;
  for (size_t i = 0; i < s.length() && out.length() < maxLen; i++) {
    char ch = s[i];
    if ((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9')) {
      out += ch;
    } else if (ch == ' ' || ch == '-' || ch == '_') {
      if (out.length() > 0 && out[out.length() - 1] != '-') out += '-';
    }
  }
  while (out.length() > 0 && out[out.length() - 1] == '-') out.remove(out.length() - 1);
  if (out.length() == 0) out = "sem-nome";
  return out;
}

bool validFileName(const String& n) {
  if (n.length() < 5 || n.length() > 48) return false;
  if (!n.endsWith(".bin")) return false;
  if (n.indexOf("..") >= 0) return false;
  for (size_t i = 0; i < n.length(); i++) {
    char ch = n[i];
    bool ok = (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '-' || ch == '_' || ch == '.';
    if (!ok) return false;
  }
  return true;
}

// ---------------------------------------------------------------- BLE (status)

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
  }
};

class WriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (c != chCmd) return;
    auto raw = c->getValue();
    String val(raw.c_str());
    val.trim();
    Serial.println("[cmd] " + val);

    if (val == "start") {
      if (!imuOk) { setStatus("imu_error"); return; }
      startRequested = true;
    } else if (val == "stop") {
      streaming = false;
      flushBatch();
    } else if (val.startsWith("rec|")) {
      // Formato: rec|<atleta>|<exercicio>|<carga_kg>
      int p1 = val.indexOf('|', 4);
      if (p1 < 0) { setStatus("rec_error|comando"); return; }
      int p2 = val.indexOf('|', p1 + 1);
      if (p2 < 0) {
        cmdAthlete  = val.substring(4, p1);
        cmdExercise = val.substring(p1 + 1);
        cmdLoadKg   = 0;
      } else {
        cmdAthlete  = val.substring(4, p1);
        cmdExercise = val.substring(p1 + 1, p2);
        String loadStr = val.substring(p2 + 1);
        loadStr.trim();
        long l = loadStr.toInt();
        if (l < 0) l = 0;
        if (l > 999) l = 999;
        cmdLoadKg = (uint16_t)l;
      }
      recCmdStart = true;
    } else if (val == "recstop") {
      recCmdStop = true;
    } else if (val == "list") {
      listRequested = true;
    } else if (val.startsWith("get|")) {
      pendingGet = val.substring(4);
      getRequested = true;
    } else if (val.startsWith("del|")) {
      pendingDel = val.substring(4);
      delRequested = true;
    }
  }
};

// ---------------------------------------------------------------- Mapeamento: gravacao

bool allocRecBuffer() {
  const uint16_t secs[] = {60, 45, 30, 20, 10};
  for (uint16_t s : secs) {
    size_t cap = (size_t)s * REC_RATE_HZ * REC_SAMPLE_BYTES;
    uint8_t* p = nullptr;
    if (psramFound()) {
      p = (uint8_t*)ps_malloc(cap);
    } else if (ESP.getFreeHeap() > cap + 40000) {
      p = (uint8_t*)malloc(cap);
    }
    if (p) {
      recBuf = p;
      recCap = cap;
      Serial.printf("[rec] buffer de %u s (%u bytes) %s\n", s, (unsigned)cap, psramFound() ? "em PSRAM" : "em RAM");
      return true;
    }
  }
  Serial.println("[rec] sem memoria para o buffer de gravacao");
  return false;
}

String saveSession(uint32_t count, String& outName) {
  size_t need = sizeof(FileHeader) + recLen;
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < need + 4096) return "sem-espaco";

  String base = String(recAthlete) + "_" + String(recExercise);
  String name = base + ".bin";
  for (int i = 2; LittleFS.exists("/" + name) && i < 100; i++) name = base + "_" + String(i) + ".bin";

  FileHeader h;
  memset(&h, 0, sizeof(h));
  memcpy(h.magic, "MPA1", 4);
  h.version = 2;
  h.sampleBytes = REC_SAMPLE_BYTES;
  h.rateHz = REC_RATE_HZ;
  h.sampleCount = count;
  h.missed = recMissed;
  h.crc32 = crc32_calc(recBuf, recLen);
  h.bootMs = recBootMs;
  for (int i = 0; i < 3; i++) h.gyroOff[i] = gyroOff[i];
  h.accRange = 0x08;
  h.gyrRange = 0x00;
  strncpy(h.athlete, recAthlete, sizeof(h.athlete) - 1);
  strncpy(h.exercise, recExercise, sizeof(h.exercise) - 1);
  h.loadKg = recLoadKg;
  h._pad = 0;

  File f = LittleFS.open("/" + name, "w");
  if (!f) return "abrir";
  size_t w = f.write((const uint8_t*)&h, sizeof(h));
  w += f.write(recBuf, recLen);
  f.close();
  if (w != need) {
    LittleFS.remove("/" + name);
    return "escrita";
  }
  outName = name;
  return "";
}

void finishRec(const char* reason) {
  recState = REC_SAVING;
  setStatus("rec_saving");
  uint32_t count = recLen / REC_SAMPLE_BYTES;
  if (count == 0) {
    recState = REC_IDLE;
    setStatus("rec_empty");
    return;
  }
  String name;
  String err = saveSession(count, name);
  if (err.length() == 0) {
    setStatus("rec_saved|" + name + "|" + String(count) + "|" + String(recMissed) + "|" + String(reason));
  } else {
    setStatus("rec_error|" + err);
  }
  recLen = 0;
  recState = REC_IDLE;
}

void handleRec() {
  if (recCmdStop && (recState == REC_IDLE)) recCmdStop = false;

  if (recCmdStart) {
    recCmdStart = false;
    if (streaming || recState != REC_IDLE) { setStatus("rec_busy"); return; }
    if (!imuOk)  { setStatus("imu_error"); return; }
    if (!fsOk)   { setStatus("fs_error"); return; }
    if (!recBuf) { setStatus("rec_nomem"); return; }

    sanitizeName(cmdAthlete, sizeof(recAthlete) - 1).toCharArray(recAthlete, sizeof(recAthlete));
    sanitizeName(cmdExercise, sizeof(recExercise) - 1).toCharArray(recExercise, sizeof(recExercise));
    recLoadKg = cmdLoadKg;

    uint32_t tCmd = millis();
    recState = REC_CALIB;
    setLed(255, 150, 0);
    setStatus("rec_calibrating|" + String((unsigned)(recCap / (REC_RATE_HZ * REC_SAMPLE_BYTES))));
    if (!calibrateGyro()) {
      recState = REC_IDLE;
      setStatus("rec_calib_moving");
      return;
    }
    recT0 = tCmd + COUNTDOWN_MS;
    recState = REC_COUNTDOWN;
    return;
  }

  static uint8_t lastCd = 255;

  if (recState == REC_COUNTDOWN) {
    if (recCmdStop) {
      recCmdStop = false;
      recState = REC_IDLE;
      lastCd = 255;
      setStatus("rec_cancelled");
      return;
    }
    int32_t remMs = (int32_t)(recT0 - millis());
    if (remMs <= 0) {
      recLen = 0;
      recMissed = 0;
      haveLast = false;
      i2cFails = 0;
      recBootMs = millis();
      lastCd = 255;
      recState = REC_RECORDING;
      setStatus("rec_recording");
    } else {
      uint8_t rem = (uint8_t)((remMs + 999) / 1000);
      if (rem != lastCd) {
        lastCd = rem;
        setStatus("rec_cd|" + String(rem));
      }
    }
    return;
  }

  if (recState == REC_RECORDING) {
    if (recCmdStop) {
      recCmdStop = false;
      finishRec("stop");
      return;
    }
    ImuSample s;
    if (bmiReadSample(s)) {
      if (haveLast) {
        uint32_t dt = (lastSensorTime - prevSensorTime) & 0xFFFFFF;
        uint32_t steps = (dt + 32) / 64;
        if (steps > 1) recMissed += steps - 1;
      }
      prevSensorTime = lastSensorTime;
      haveLast = true;

      int16_t v[6] = {s.gx, s.gy, s.gz, s.ax, s.ay, s.az};
      memcpy(recBuf + recLen, v, REC_SAMPLE_BYTES);
      recLen += REC_SAMPLE_BYTES;
      if (recLen + REC_SAMPLE_BYTES > recCap) finishRec("full");
    }
    if (i2cFails > 50) {
      i2cFails = 0;
      finishRec("i2c");
    }
  }
}

// ---------------------------------------------------------------- Mapeamento: arquivos

void sendFileList() {
  setStatus("fs|" + String((unsigned)LittleFS.usedBytes()) + "|" + String((unsigned)LittleFS.totalBytes()));
  delay(20);
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      String n = f.name();
      if (n.startsWith("/")) n = n.substring(1);
      FileHeader h;
      bool ok = f.size() >= sizeof(FileHeader) &&
                f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) &&
                memcmp(h.magic, "MPA1", 4) == 0;
      String line = "file|" + n + "|" + String((unsigned)f.size()) + "|";
      if (ok) {
        h.athlete[sizeof(h.athlete) - 1] = 0;
        h.exercise[sizeof(h.exercise) - 1] = 0;
        line += String(h.sampleCount) + "|" + String(h.missed) + "|" + String(h.athlete) + "|" + String(h.exercise);
      } else {
        line += "0|0||";
      }
      setStatus(line);
      delay(20);
    }
    f = root.openNextFile();
  }
  setStatus("list_end");
}

void sendFile(const String& name) {
  if (!validFileName(name)) { setStatus("get_error|" + name); return; }
  File f = LittleFS.open("/" + name, "r");
  if (!f) { setStatus("get_error|" + name); return; }
  size_t size = f.size();
  setStatus("xfer_start|" + name + "|" + String((unsigned)size));
  delay(30);

  uint8_t pkt[4 + XFER_CHUNK];
  uint32_t off = 0;
  while (off < size && connected) {
    size_t n = f.read(pkt + 4, XFER_CHUNK);
    if (n == 0) break;
    memcpy(pkt, &off, 4);
    chXfer->setValue(pkt, 4 + n);
    chXfer->notify();
    off += n;
    delay(XFER_DELAY_MS);
  }
  f.close();
  delay(30);
  setStatus("xfer_end|" + name + "|" + String(off));
}

void deleteFile(const String& name) {
  if (validFileName(name) && LittleFS.remove("/" + name)) setStatus("del_ok|" + name);
  else setStatus("del_error|" + name);
}

void handleFiles() {
  if (recState != REC_IDLE || streaming) return;
  if (listRequested) { listRequested = false; sendFileList(); }
  if (getRequested)  { getRequested = false;  sendFile(pendingGet); }
  if (delRequested)  { delRequested = false;  deleteFile(pendingDel); }
}

// ---------------------------------------------------------------- LED

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  static uint32_t last = 0xFFFFFFFF;
  uint32_t c = strip.Color(r, g, b);
  if (c == last) return;
  last = c;
  strip.setPixelColor(0, c);
  strip.show();
}

void updateLed() {
  static uint32_t lastToggle = 0;
  static bool on = false;
  uint32_t now = millis();

  if (!connected && recState != REC_RECORDING) {
    if (now - lastToggle >= 250) { lastToggle = now; on = !on; setLed(0, 0, on ? 255 : 0); }
  } else if (recState == REC_COUNTDOWN) {
    if (now - lastToggle >= 250) { lastToggle = now; on = !on; setLed(on ? 255 : 0, on ? 150 : 0, 0); }
  } else if (streaming || recState == REC_RECORDING) {
    if (now - lastToggle >= 250) { lastToggle = now; on = !on; setLed(on ? 255 : 0, 0, 0); }
  } else if (recState == REC_IDLE) {
    setLed(0, 255, 0);
  }
}

// ---------------------------------------------------------------- modo ao vivo

void handleLiveStart() {
  if (!startRequested) return;
  startRequested = false;
  if (recState != REC_IDLE) { setStatus("rec_busy"); return; }
  streaming = false;
  setLed(255, 150, 0);
  setStatus("calibrating");
  if (calibrateGyro()) {
    sampleSeq = 0;
    batchSeq  = 0;
    batchFill = 0;
    i2cFails  = 0;
    streaming = true;
    setStatus("streaming");
  } else {
    setStatus("calib_moving");
  }
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  Serial.begin(115200);
  delay(1500);
  strip.begin();
  strip.show();

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

  fsOk = LittleFS.begin(true);
  if (fsOk) {
    Serial.printf("[fs] LittleFS OK: %u de %u bytes usados\n", (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
  } else {
    Serial.println("[fs] FALHA ao montar LittleFS (confira o Partition Scheme)");
  }

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
  chImu  = data->createCharacteristic(CHR_IMU_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  chXfer = data->createCharacteristic(CHR_XFER_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  chImu->addDescriptor(new BLE2902());
  chXfer->addDescriptor(new BLE2902());
  data->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_PROV_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  allocRecBuffer();

  setStatus(imuOk ? "idle" : "imu_error");
}

void loop() {
  updateLed();
  handleLiveStart();
  handleRec();
  handleFiles();

  if (streaming && connected) {
    ImuSample s;
    if (bmiReadSample(s)) {
      s.gx -= gyroOff[0];
      s.gy -= gyroOff[1];
      s.gz -= gyroOff[2];
      s.t_ms = millis();
      s.seq  = sampleSeq++;
      batch.samples[batchFill++] = s;
      if (batchFill == BATCH_SIZE) flushBatch();
    }

    if (i2cFails > 50) {
      streaming = false;
      flushBatch();
      setStatus("imu_error");
    }
  }
}