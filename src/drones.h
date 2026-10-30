// Dekoder Open Drone ID (ASTM F3411) na płytce i tabela dronów w zasięgu.
// Potrzebny do ekranu, oznaczeń swój/obcy i alarmów LoRa (Meshtastic) bez komputera.
// Dołączany z main.cpp po definicji struktury Frame.
#pragma once

#define MAX_DRONES 16
#define DRONE_FORGET_MS 120000  // po tylu ms bez ramki slot może zająć inny dron
#define ODID_EPOCH_2019 1546300800UL

struct DroneInfo {
  bool used;
  char id[21];
  uint8_t macs[3][6];
  uint8_t nmac;
  int8_t rssi;
  uint32_t firstMs, lastMs;
  bool hasLoc;
  int32_t lat, lon;  // 1e-7 stopnia
  float altGeo, height, speed;  // NAN = brak
  int16_t track;                // -1 = brak
  uint8_t status;               // 0 nieznany, 1 na ziemi, 2 w powietrzu, 3 awaria
  bool hasOp;
  int32_t opLat, opLon;
  char opId[21];  // numer operatora (Operator ID), pusty = brak
  float vspeed, altBaro, opAlt;  // NAN = brak
  uint8_t uaType, opLocType;     // kody ASTM; opLocType: 0 start, 1 na żywo, 2 stała
  uint8_t euCat, euClass;        // 0 = nieokreślone; euClass: 1 = C0 ... 7 = C6
  char selfId[24];               // opis ustawiony przez operatora
  float maxHeight, maxSpeed;     // maksima w tym uruchomieniu płytki; NAN = brak
  // alarmy LoRa
  bool alerted;
  bool shown;  // nowy dron już obudził ekran
  uint32_t lastWaypointMs;
  // dron Wi-Fi bez Remote ID (wifidrone.h): rodzaj, kanał; id = nazwa sieci
  const char *wifiKind;  // nullptr = dron Remote ID
  uint8_t wifiCh;
  uint32_t wifiPrintMs;
};

static DroneInfo drones[MAX_DRONES];
// czas Unix = unixBase + millis()/1000; 0 = nieznany (ustawia TIME z PC albo System drona)
static uint32_t unixBase = 0;

static uint32_t unixNow() { return unixBase ? unixBase + millis() / 1000 : 0; }

// ---------------------------------------------------------------- oznaczenia swój / obcy

enum TagStatus : uint8_t { TAG_NONE = 0, TAG_OWN = 1, TAG_FOREIGN = 2 };
struct TagEntry {
  char id[21];
  uint8_t status;
};
#define MAX_TAGS 32
static TagEntry tags[MAX_TAGS];

static uint8_t tagStatus(const char *id) {
  if (!id[0]) return TAG_NONE;
  for (auto &t : tags)
    if (t.status && strcmp(t.id, id) == 0) return t.status;
  return TAG_NONE;
}

static const char *tagText(uint8_t st) { return st == TAG_OWN ? "SWOJ" : st == TAG_FOREIGN ? "OBCY" : ""; }

// ---------------------------------------------------------------- tabela dronów

static int32_t rdI32(const uint8_t *p) {
  int32_t v;
  memcpy(&v, p, 4);
  return v;
}
static uint16_t rdU16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static float odidAlt(const uint8_t *p) {
  uint16_t v = rdU16(p);
  return v ? v * 0.5f - 1000.0f : NAN;
}

static bool macIn(const DroneInfo &d, const uint8_t *mac) {
  for (int i = 0; i < d.nmac; i++)
    if (memcmp(d.macs[i], mac, 6) == 0) return true;
  return false;
}

static void addMac(DroneInfo &d, const uint8_t *mac) {
  if (macIn(d, mac)) return;
  if (d.nmac < 3) memcpy(d.macs[d.nmac++], mac, 6);
  else memcpy(d.macs[2], mac, 6);  // BLE zmienia adresy - trzymamy ostatni
}

static DroneInfo *droneForMac(const uint8_t *mac) {
  uint32_t now = millis();
  int freeSlot = -1, oldest = 0;
  for (int i = 0; i < MAX_DRONES; i++) {
    if (drones[i].used && macIn(drones[i], mac)) return &drones[i];
    if (!drones[i].used || now - drones[i].lastMs > DRONE_FORGET_MS) {
      if (freeSlot < 0) freeSlot = i;
    } else if (drones[i].lastMs < drones[oldest].lastMs) {
      oldest = i;
    }
  }
  DroneInfo &d = drones[freeSlot >= 0 ? freeSlot : oldest];
  memset(&d, 0, sizeof(d));
  d.used = true;
  d.firstMs = now;
  d.altGeo = d.height = d.speed = d.vspeed = d.altBaro = d.opAlt = d.maxHeight = d.maxSpeed = NAN;
  d.track = -1;
  addMac(d, mac);
  return &d;
}

// ten sam dron nadający przez Wi-Fi i BLE (inne MAC) - łączymy po UAS ID
static DroneInfo *mergeById(DroneInfo *d) {
  for (auto &o : drones) {
    if (&o == d || !o.used || strcmp(o.id, d->id) != 0) continue;
    for (int i = 0; i < d->nmac; i++) addMac(o, d->macs[i]);
    if (d->hasLoc) {
      o.hasLoc = true;
      o.lat = d->lat; o.lon = d->lon; o.altGeo = d->altGeo; o.height = d->height;
      o.speed = d->speed; o.track = d->track; o.status = d->status;
    }
    if (d->hasOp && !o.hasOp) { o.hasOp = true; o.opLat = d->opLat; o.opLon = d->opLon; }
    if (d->opId[0] && !o.opId[0]) strcpy(o.opId, d->opId);
    if (d->selfId[0] && !o.selfId[0]) strcpy(o.selfId, d->selfId);
    if (d->uaType && !o.uaType) o.uaType = d->uaType;
    if (d->euClass && !o.euClass) { o.euClass = d->euClass; o.euCat = d->euCat; }
    if (!isnan(d->opAlt) && isnan(o.opAlt)) { o.opAlt = d->opAlt; o.opLocType = d->opLocType; }
    if (!isnan(d->maxHeight) && !(o.maxHeight >= d->maxHeight)) o.maxHeight = d->maxHeight;
    if (!isnan(d->maxSpeed) && !(o.maxSpeed >= d->maxSpeed)) o.maxSpeed = d->maxSpeed;
    o.lastMs = max(o.lastMs, d->lastMs);
    o.firstMs = min(o.firstMs, d->firstMs);
    d->used = false;
    return &o;
  }
  return d;
}

static void parseMessage(DroneInfo *&d, const uint8_t *m) {
  switch (m[0] >> 4) {
    case 0: {  // Basic ID
      char id[21];
      memcpy(id, m + 2, 20);
      id[20] = 0;
      for (int i = 19; i >= 0 && (id[i] == 0 || id[i] == ' '); i--) id[i] = 0;
      for (int i = 0; id[i]; i++)
        if (id[i] < 32 || id[i] > 126) id[i] = '?';
      if (m[1] & 0x0F) d->uaType = m[1] & 0x0F;
      // pierwsze ID albo numer seryjny (typ 1) wygrywa z ID sesji
      if (id[0] && (!d->id[0] || (m[1] >> 4) == 1)) {
        strcpy(d->id, id);
        d = mergeById(d);
      }
      break;
    }
    case 1: {  // Location / Vector
      uint8_t flags = m[1];
      d->status = flags >> 4;
      int dir = m[2] + ((flags & 0x02) ? 180 : 0);
      d->track = dir <= 360 ? dir : -1;
      uint8_t sp = m[3];
      d->speed = sp == 255 ? NAN : (flags & 0x01) ? sp * 0.75f + 63.75f : sp * 0.25f;
      int32_t lat = rdI32(m + 5), lon = rdI32(m + 9);
      if (lat || lon) {
        d->hasLoc = true;
        d->lat = lat;
        d->lon = lon;
      }
      d->altGeo = odidAlt(m + 15);
      d->height = odidAlt(m + 17);
      d->altBaro = odidAlt(m + 13);
      d->vspeed = (int8_t)m[4] == 126 ? NAN : (int8_t)m[4] * 0.5f;
      if (!isnan(d->height) && !(d->maxHeight >= d->height)) d->maxHeight = d->height;
      if (!isnan(d->speed) && !(d->maxSpeed >= d->speed)) d->maxSpeed = d->speed;
      break;
    }
    case 3: {  // Self ID: opis; znaki spoza ASCII i niebezpieczne w JSON zastępujemy
      char txt[24];
      memcpy(txt, m + 2, 23);
      txt[23] = 0;
      for (int i = 22; i >= 0 && (txt[i] == 0 || txt[i] == ' '); i--) txt[i] = 0;
      for (int i = 0; txt[i]; i++)
        if (txt[i] < 32 || txt[i] > 126 || txt[i] == '"' || txt[i] == '\\') txt[i] = '?';
      strcpy(d->selfId, txt);
      break;
    }
    case 4: {  // System: pozycja operatora i czas
      int32_t lat = rdI32(m + 2), lon = rdI32(m + 6);
      if (lat || lon) {
        d->hasOp = true;
        d->opLat = lat;
        d->opLon = lon;
      }
      d->opLocType = m[1] & 0x03;
      d->opAlt = odidAlt(m + 18);
      if (((m[1] >> 2) & 0x07) == 1) {  // klasyfikacja UE
        d->euCat = m[17] >> 4;
        d->euClass = m[17] & 0x0F;
      }
      uint32_t ts;
      memcpy(&ts, m + 20, 4);
      if (!unixBase && ts) unixBase = ODID_EPOCH_2019 + ts - millis() / 1000;
      break;
    }
    case 5: {  // Operator ID: tylko znaki bezpieczne w JSON
      char id[21];
      memcpy(id, m + 2, 20);
      id[20] = 0;
      for (int i = 19; i >= 0 && (id[i] == 0 || id[i] == ' '); i--) id[i] = 0;
      for (int i = 0; id[i]; i++)
        if (!isalnum((unsigned char)id[i]) && id[i] != '-' && id[i] != '_' && id[i] != '.') id[i] = '?';
      if (id[0]) strcpy(d->opId, id);
      break;
    }
  }
}

// zwraca drona, którego dotyczyła ramka (albo nullptr)
static DroneInfo *parseFrame(const Frame &f) {
  // Wi-Fi Beacon: OUI(3) typ(1) licznik(1); BLE: app code(1) licznik(1); NAN: licznik(1)
  size_t skip = f.transport == T_BEACON ? 5 : f.transport == T_BLE ? 2 : 1;
  if (f.len < skip + 25) return nullptr;
  const uint8_t *m = f.data + skip;
  size_t len = f.len - skip;
  DroneInfo *d = droneForMac(f.mac);
  d->lastMs = millis();
  d->rssi = f.rssi;
  size_t count = 1, step = 25;
  if ((m[0] >> 4) == 0xF && len >= 3) {  // Message Pack
    step = m[1] ? m[1] : 25;
    count = m[2];
    m += 3;
    len -= 3;
  }
  for (size_t k = 0; k < count && (k + 1) * step <= len && step >= 25; k++) parseMessage(d, m + k * step);
  return d;
}

// odległość w metrach (przybliżenie równoprostokątne - wystarcza do kilkunastu km)
static float distM(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float la = (lat1 + lat2) * 0.5e-7f * DEG_TO_RAD;
  float dx = (lon2 - lon1) * 1e-7f * DEG_TO_RAD * cosf(la);
  float dy = (lat2 - lat1) * 1e-7f * DEG_TO_RAD;
  return 6371000.0f * sqrtf(dx * dx + dy * dy);
}
