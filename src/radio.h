// Odbiór ramek Open Drone ID (jak w firmware Heltec): Wi-Fi Beacon, Wi-Fi NAN, BLE 4.
// ESP32 na CYD ma Bluetooth 4.2 - bez BLE 5 Long Range (Coded PHY).
// Do tego drony Wi-Fi bez Remote ID (wifidrone.h) i statystyka zajętości kanałów 1-13.
// Dołączany z main.cpp po definicji Frame/enqueue i po wifidrone.h.
#pragma once

#include <WiFi.h>
#include <esp_wifi.h>
#include <NimBLEDevice.h>

static const uint8_t ODID_OUI[3] = {0xFA, 0x0B, 0xBC};
static const uint8_t WFA_OUI[3] = {0x50, 0x6F, 0x9A};
static const uint8_t ODID_NAN_SERVICE_ID[6] = {0x88, 0x69, 0x19, 0x9D, 0x92, 0x09};  // SHA-256("org.opendroneid.remoteid")[0..5]

static volatile uint8_t currentChannel = 6;
static volatile uint8_t fixedChannel = 0;  // 0 = skakanie
static volatile uint32_t allBeacon = 0, allBle = 0;

// zajętość kanałów Wi-Fi: ramki i najsilniejszy sygnał od ostatniego podsumowania, czas nasłuchu kanału
static volatile uint32_t chFrames[14], chDwellMs[14];
static volatile int8_t chPeak[14];
static uint16_t chRate[14];      // ramek na sekundę (ostatnie podsumowanie)
static int8_t chRssi[14];        // najsilniejszy sygnał [dBm], -128 = brak danych
static uint32_t chSummaryMs = 0;

// co ~10 s nasłuchu: ramki/s na kanał (kanały poza 6 odwiedzamy rzadziej, więc dzielimy przez czas)
static void chSummaryTick() {
  uint32_t total = 0;
  for (int c = 1; c <= 13; c++) total += chDwellMs[c];
  if (total < 10000) return;
  for (int c = 1; c <= 13; c++) {
    if (!chDwellMs[c]) continue;
    chRate[c] = chFrames[c] * 1000 / chDwellMs[c];
    chRssi[c] = chFrames[c] ? chPeak[c] : -128;
    chFrames[c] = chDwellMs[c] = 0;
    chPeak[c] = -128;
  }
  chSummaryMs = millis();
}

// true = beacon z Remote ID
static bool handleBeacon(const uint8_t *p, size_t len, int8_t rssi) {
  size_t i = 36;  // nagłówek 24 B + timestamp 8 + interwał 2 + capability 2
  while (i + 2 <= len) {
    uint8_t id = p[i], ln = p[i + 1];
    if (i + 2 + ln > len) break;
    const uint8_t *body = p + i + 2;
    if (id == 221 && ln > 5 && memcmp(body, ODID_OUI, 3) == 0 && body[3] == 0x0D) {
      enqueue(T_BEACON, p + 10, rssi, body, ln);
      wdNoteOdid(p + 10);
      return true;
    }
    i += 2 + ln;
  }
  return false;
}

static void handleAction(const uint8_t *p, size_t len, int8_t rssi) {
  // Public Action (4), Vendor Specific (9), OUI WFA, typ 0x13 = NAN Service Discovery
  if (len < 31) return;
  const uint8_t *b = p + 24;
  if (!(b[0] == 4 && b[1] == 9 && memcmp(b + 2, WFA_OUI, 3) == 0 && b[5] == 0x13)) return;
  size_t i = 30;
  while (i + 3 <= len) {
    uint8_t attr = p[i];
    uint16_t alen = p[i + 1] | (p[i + 2] << 8);
    if (i + 3 + alen > len) break;
    const uint8_t *a = p + i + 3;
    if (attr == 0x03 && alen >= 9) {  // Service Descriptor Attribute
      uint8_t ctrl = a[8];
      size_t k = 9;
      if (ctrl & 0x40) k += 2;
      if ((ctrl & 0x04) && k < alen) k += 1 + a[k];
      if ((ctrl & 0x08) && k < alen) k += 1 + a[k];
      if ((ctrl & 0x10) && k < alen) {  // Service Info
        uint8_t silen = a[k];
        const uint8_t *si = a + k + 1;
        if (k + 1 + silen > alen || silen < 4) return;
        bool idMatch = memcmp(a, ODID_NAN_SERVICE_ID, 6) == 0;
        bool packLike = (si[1] >> 4) == 0xF && si[2] == 25;
        if (idMatch || packLike) enqueue(T_NAN, p + 10, rssi, si, silen);
      }
      return;
    }
    i += 3 + alen;
  }
}

static void IRAM_ATTR wifiSniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
  auto *pkt = (const wifi_promiscuous_pkt_t *)buf;
  uint8_t ch = currentChannel;
  if (ch <= 13) {
    chFrames[ch]++;
    if (pkt->rx_ctrl.rssi > chPeak[ch]) chPeak[ch] = pkt->rx_ctrl.rssi;
  }
  if (type != WIFI_PKT_MGMT) return;
  const uint8_t *p = pkt->payload;
  size_t len = pkt->rx_ctrl.sig_len;
  if (len > 4) len -= 4;  // FCS
  if (len < 24) return;
  int8_t rssi = pkt->rx_ctrl.rssi;
  if (p[0] == 0x80) {
    allBeacon++;
    if (!handleBeacon(p, len, rssi)) wdCheck(p, len, rssi, ch);
  } else if (p[0] == 0x50) {
    wdCheck(p, len, rssi, ch);  // odpowiedź na sondowanie (dron Wi-Fi)
  } else if (p[0] == 0xD0) {
    handleAction(p, len, rssi);
  }
}

// kanał 6 (NAN, najczęstszy dla RID) większość czasu, co chwilę pozostałe kanały 2,4 GHz
static void channelTask(void *) {
  static const uint8_t others[] = {1, 11, 2, 7, 3, 8, 4, 9, 5, 10, 12, 13};
  size_t idx = 0;
  for (;;) {
    if (fixedChannel) {
      if (currentChannel != fixedChannel) {
        currentChannel = fixedChannel;
        esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      chDwellMs[currentChannel] += 200;
      continue;
    }
    currentChannel = 6;
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(pdMS_TO_TICKS(600));
    chDwellMs[6] += 600;
    if (fixedChannel) continue;
    currentChannel = others[idx];
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    idx = (idx + 1) % sizeof(others);
    vTaskDelay(pdMS_TO_TICKS(150));
    chDwellMs[currentChannel] += 150;
  }
}

class OdidScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice *dev) override {
    allBle++;
    const uint8_t *p = dev->getPayload();
    size_t len = dev->getPayloadLength();
    size_t i = 0;
    while (i + 1 < len) {
      uint8_t ln = p[i];
      if (ln == 0 || i + 1 + ln > len) break;
      uint8_t type = p[i + 1];
      const uint8_t *d = p + i + 2;
      // Service Data 16-bit UUID 0xFFFA, app code 0x0D
      if (type == 0x16 && ln >= 5 && d[0] == 0xFA && d[1] == 0xFF && d[2] == 0x0D) {
        const uint8_t *a = dev->getAddress().getNative();
        uint8_t mac[6] = {a[5], a[4], a[3], a[2], a[1], a[0]};
        enqueue(T_BLE, mac, dev->getRSSI(), d + 2, ln - 3);
        return;
      }
      i += 1 + ln;
    }
  }
};

static volatile bool bleEnabled = true;  // false = tylko Wi-Fi (całe radio dla Wi-Fi, mniejszy pobór)

static void restartScan(NimBLEScanResults) {
  if (bleEnabled) NimBLEDevice::getScan()->start(0, restartScan, false);
}

static void bleSetEnabled(bool on) {
  bleEnabled = on;
  if (on) NimBLEDevice::getScan()->start(0, restartScan, false);
  else NimBLEDevice::getScan()->stop();
}

static void radioBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  for (int c = 0; c < 14; c++) chPeak[c] = chRssi[c] = -128;
  wdBegin();
  // ramki danych tylko do statystyki zajętości kanałów
  wifi_promiscuous_filter_t filt = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA};
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&wifiSniffer);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  xTaskCreatePinnedToCore(channelTask, "chan", 2048, nullptr, 1, nullptr, 0);

  NimBLEDevice::init("");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new OdidScanCallbacks(), true);
  scan->setActiveScan(false);
  scan->setDuplicateFilter(false);
  scan->setMaxResults(0);
  scan->setInterval(100);
  scan->setWindow(50);  // reszta czasu dla Wi-Fi (wspólne radio)
  if (bleEnabled) scan->start(0, restartScan, false);
}
