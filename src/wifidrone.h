// Drony sterowane przez Wi-Fi (bez Remote ID): zabawkowe drony z kamerą Wi-Fi, DJI Tello, Parrot,
// starsze DJI w trybie Wi-Fi. Taki dron jest punktem dostępowym, więc rozpoznajemy go po nazwie
// sieci (SSID) albo producencie (OUI) w beaconach i odpowiedziach na sondowanie.
// Wspólny plik dla Heltec, CYD i M5 - dołączany po drones.h, przed obsługą Wi-Fi.
// Wynik trafia do tej samej tabeli co drony Remote ID (id = nazwa sieci, wifiKind = rodzaj),
// więc lista, oznaczenia swój/obcy i alarmy działają bez zmian.
#pragma once

struct WdEvent {
  uint8_t mac[6];
  char ssid[21];
  const char *kind;
  uint8_t ch;
  int8_t rssi;
};

static QueueHandle_t wdQueue;
static volatile uint32_t wdSeen = 0;  // ile zdarzeń dron Wi-Fi (diagnostyka)

// nazwa sieci: początek (prefix) albo dowolne miejsce (any), bez rozróżniania wielkości liter
static const struct { const char *pat; bool any; const char *kind; } WD_SSID[] = {
    {"TELLO-", false, "DJI Tello"},      {"RMTT-", false, "DJI RoboMaster TT"},
    {"Spark-", false, "DJI Spark"},      {"MavicAir", false, "DJI Mavic Air"},
    {"Mavic-", false, "DJI Mavic Wi-Fi"}, {"PHANTOM", false, "DJI Phantom"},
    {"Anafi", false, "Parrot Anafi"},    {"Bebop", false, "Parrot Bebop"},
    {"Disco-", false, "Parrot Disco"},   {"Mambo", false, "Parrot Mambo"},
    {"Skydio", false, "Skydio"},         {"FIMI", false, "Xiaomi FIMI"},
    {"Breeze", false, "Yuneec Breeze"},  {"Potensic", false, "Potensic"},
    {"SNAPTAIN", false, "Snaptain"},     {"Eachine", false, "Eachine"},
    {"Hubsan", false, "Hubsan"},         {"HolyStone", false, "Holy Stone"},
    {"JJRC", false, "JJRC"},             {"SYMA", false, "Syma"},
    {"WIFI-UFO", false, "dron zabawkowy"}, {"WIFI_UFO", false, "dron zabawkowy"},
    {"WiFi-720P", false, "dron zabawkowy"}, {"HD-720P", false, "dron zabawkowy"},
    {"HD-1080P", false, "dron zabawkowy"}, {"WIFI_FPV", false, "dron FPV Wi-Fi"},
    {"FPV-", false, "dron FPV Wi-Fi"},   {"FPV_", false, "dron FPV Wi-Fi"},
    {"drone", true, "nazwa: drone"},     {"quadcopter", true, "nazwa: quadcopter"},
    {"UAV", true, "nazwa: UAV"},         {"UFO-", false, "dron zabawkowy"},
};

// producenci dronów - tylko punkty dostępowe (beacon / odpowiedź na sondowanie)
static const struct { uint8_t oui[3]; const char *kind; } WD_OUI[] = {
    {{0x60, 0x60, 0x1F}, "DJI Wi-Fi"}, {{0x34, 0xD2, 0x62}, "DJI Wi-Fi"}, {{0x48, 0x1C, 0xB9}, "DJI Wi-Fi"},
    {{0x58, 0xB8, 0x58}, "DJI Wi-Fi"}, {{0xE4, 0x7A, 0x2C}, "DJI Wi-Fi"}, {{0x0C, 0x9A, 0xE6}, "DJI Wi-Fi"},
    {{0x04, 0xA8, 0x5A}, "DJI Wi-Fi"}, {{0x90, 0x03, 0xB7}, "Parrot"},    {{0xA0, 0x14, 0x3D}, "Parrot"},
    {{0x00, 0x12, 0x1C}, "Parrot"},    {{0x00, 0x26, 0x7E}, "Parrot"},    {{0x90, 0x3A, 0xE6}, "Parrot"},
};

// MAC-i, które nadają Remote ID - tego drona już widać jako RID, nie dublujemy alarmu
static uint8_t wdOdidMacs[8][6];
static uint8_t wdOdidNext = 0;

static void wdNoteOdid(const uint8_t *mac) {
  for (auto &m : wdOdidMacs)
    if (memcmp(m, mac, 6) == 0) return;
  memcpy(wdOdidMacs[wdOdidNext], mac, 6);
  wdOdidNext = (wdOdidNext + 1) % 8;
}

static bool wdIsOdid(const uint8_t *mac) {
  for (auto &m : wdOdidMacs)
    if (memcmp(m, mac, 6) == 0) return true;
  return false;
}

static bool wdMatchAt(const char *s, const char *p) {
  for (; *p; s++, p++)
    if (!*s || tolower((unsigned char)*s) != tolower((unsigned char)*p)) return false;
  return true;
}

static const char *wdKindFor(const uint8_t *mac, const char *ssid) {
  for (auto &w : WD_SSID) {
    if (!w.any) {
      if (wdMatchAt(ssid, w.pat)) return w.kind;
    } else {
      for (const char *s = ssid; *s; s++)
        if (wdMatchAt(s, w.pat)) return w.kind;
    }
  }
  for (auto &o : WD_OUI)
    if (memcmp(mac, o.oui, 3) == 0) return o.kind;
  return nullptr;
}

// wywoływane z odbioru Wi-Fi dla beaconów (0x80) i odpowiedzi na sondowanie (0x50) bez Remote ID
static void wdCheck(const uint8_t *p, size_t len, int8_t rssi, uint8_t ch) {
  static uint8_t lastMac[6];
  static uint32_t lastMs = 0;
  const uint8_t *mac = p + 10;  // adres nadawcy
  if (len < 38 || p[36] != 0 || wdIsOdid(mac)) return;  // pierwszy element to SSID
  uint8_t sl = p[37];
  if (sl > 32 || 38 + sl > len) return;
  char ssid[33];
  bool hidden = true;
  for (int i = 0; i < sl; i++) {
    char c = p[38 + i];
    if (c) hidden = false;
    ssid[i] = (c < 32 || c > 126 || c == '"' || c == '\\') ? '?' : c;
  }
  ssid[hidden ? 0 : sl] = 0;
  const char *kind = wdKindFor(mac, ssid);
  if (!kind) return;
  uint32_t now = millis();
  if (memcmp(mac, lastMac, 6) == 0 && now - lastMs < 500) return;  // ten sam dron co chwilę
  memcpy(lastMac, mac, 6);
  lastMs = now;
  WdEvent e;
  memcpy(e.mac, mac, 6);
  if (ssid[0]) strlcpy(e.ssid, ssid, sizeof(e.ssid));
  else snprintf(e.ssid, sizeof(e.ssid), "WIFI-%02X%02X%02X", mac[3], mac[4], mac[5]);
  e.kind = kind;
  e.ch = ch;
  e.rssi = rssi;
  if (xQueueSend(wdQueue, &e, 0) == pdTRUE) wdSeen++;
}

static void wdBegin() { wdQueue = xQueueCreate(8, sizeof(WdEvent)); }

// pętla główna: zdarzenie -> wpis w tabeli dronów (zwraca drona; nowy ma shown == false)
static DroneInfo *wdApply(const WdEvent &e) {
  DroneInfo *d = droneForMac(e.mac);
  d->lastMs = millis();
  d->rssi = e.rssi;
  d->wifiKind = e.kind;
  d->wifiCh = e.ch;
  strlcpy(d->id, e.ssid, sizeof(d->id));
  return d;
}

// linia dla agenta PC (najwyżej co 3 s na drona): {"wd":{"mac":..,"ssid":..,"kind":..,"ch":6,"rssi":-50}}
static void wdPrint(Print &out, DroneInfo &d) {
  uint32_t now = millis();
  if (d.wifiPrintMs && now - d.wifiPrintMs < 3000) return;
  d.wifiPrintMs = now;
  const uint8_t *m = d.macs[0];
  out.printf("{\"wd\":{\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"ssid\":\"%s\",\"kind\":\"%s\",\"ch\":%u,\"rssi\":%d}}\n",
             m[0], m[1], m[2], m[3], m[4], m[5], d.id, d.wifiKind, d.wifiCh, d.rssi);
}
