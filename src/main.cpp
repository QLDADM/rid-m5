// Osobisty alarm Remote ID na M5StickC Plus2.
//
// Odbiera ramki Open Drone ID (Wi-Fi Beacon, Wi-Fi NAN, BLE 4), dekoduje je i piszczy przy nowym
// dronie, który nie jest oznaczony jako swój (oznaczenia jak na Heltec/CYD/stronie RID Decoder).
// Widoki (przycisk A - przód):  ALARM -> DRONY -> SZUKAJ -> USTAWIENIA
//   B (bok)          - następny dron (DRONY / SZUKAJ), na ALARM - lista dronów, USTAWIENIA - następna opcja
//   A przytrzymaj    - DRONY: oznaczenie brak -> SWOJ -> OBCY
//   B przytrzymaj    - USTAWIENIA: zmiana opcji (dźwięk, Bluetooth, kanał), w innych widokach dźwięk wł./wył.
//   zasilanie krótko - ekran wł./wył.,  przytrzymaj - wyłączenie
// SZUKAJ: piszczy tym szybciej, im silniejszy sygnał wybranego drona (chodzenie "na ciepło-zimno").
// Przez USB-C (921600 bod) mówi protokołem jak Heltec ("dev":"m5"), agent RID Decoder rozpoznaje go sam.

#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>

#define PIN_LED 19
#define DIM_AFTER_MS 20000
#define SLEEP_AFTER_MS 60000
#define ACTIVE_MS 60000       // dron "w pobliżu", gdy ostatnia ramka młodsza niż minuta
#define REMIND_MS 60000       // przypomnienie dźwiękiem, gdy obcy dron nadal w pobliżu
#define BRIGHT_ON 110
#define BRIGHT_DIM 15

enum Transport : uint8_t { T_BEACON, T_NAN, T_BLE };
static const char *TRANSPORT_NAMES[] = {"wifi_beacon", "wifi_nan", "ble"};

struct Frame {
  uint8_t transport;
  uint8_t channel;
  int8_t rssi;
  uint8_t mac[6];
  uint16_t len;
  uint8_t data[256];
};

static QueueHandle_t frameQueue;
static volatile uint32_t counts[3] = {0, 0, 0};
static volatile uint32_t dropped = 0;
static Preferences prefs;
static uint16_t bootId = 0;

#define FW_VERSION "1.5"  // wspólny numer dla Heltec, CYD i M5 - nowy tylko przy dużych zmianach
#include "drones.h"
#include "wifidrone.h"  // drony Wi-Fi bez Remote ID (Tello, Parrot, zabawkowe z kamerą Wi-Fi)

static void enqueue(uint8_t transport, const uint8_t *mac, int8_t rssi, const uint8_t *data, size_t len);

#include "radio.h"

static void enqueue(uint8_t transport, const uint8_t *mac, int8_t rssi, const uint8_t *data, size_t len) {
  if (len > sizeof(Frame::data)) return;
  Frame f;
  f.transport = transport;
  f.channel = transport == T_BLE ? 0 : currentChannel;
  f.rssi = rssi;
  memcpy(f.mac, mac, 6);
  f.len = len;
  memcpy(f.data, data, len);
  if (xQueueSend(frameQueue, &f, 0) == pdTRUE) counts[transport]++;
  else dropped++;
}

// ---------------------------------------------------------------- stan

enum View : uint8_t { V_ALARM, V_DRONES, V_HUNT, V_SETTINGS };
static uint8_t setSel = 0;
static View view = V_ALARM;
static uint8_t droneIdx = 0;
static char selId[21] = "";
static bool soundOn = true, screenOn = true, dimmed = false;
static uint32_t lastInputMs = 0, lastRemindMs = 0, lastHuntBeep = 0, ledUntil = 0, lastHostMs = 0;
static int8_t huntMax = -127;
static bool dirty = true;
static char toast[32];
static uint32_t toastUntil = 0;

#define MAX_TAG_PENDING 16
static TagEntry tagPending[MAX_TAG_PENDING];
static uint8_t tagPendingCount = 0;

// ---------------------------------------------------------------- czas (RTC trzyma UTC)

static uint32_t unixFromRtc() {
  if (!M5.Rtc.isEnabled()) return 0;
  auto dt = M5.Rtc.getDateTime();
  if (dt.date.year < 2024) return 0;  // zegar nieustawiony
  int y = dt.date.year, m = dt.date.month, d = dt.date.date;
  y -= m <= 2;  // days_from_civil (H. Hinnant)
  int era = y / 400;
  unsigned yoe = y - era * 400, doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + doe - 719468;
  return days * 86400UL + dt.time.hours * 3600UL + dt.time.minutes * 60UL + dt.time.seconds;
}

static void setClock(uint32_t unix) {
  unixBase = unix - millis() / 1000;
  time_t t = unix;
  struct tm g;
  gmtime_r(&t, &g);
  M5.Rtc.setDateTime(m5::rtc_datetime_t{{(int16_t)(g.tm_year + 1900), (int8_t)(g.tm_mon + 1), (int8_t)g.tm_mday},
                                        {(int8_t)g.tm_hour, (int8_t)g.tm_min, (int8_t)g.tm_sec}});
}

static String clockText() {
  uint32_t now = unixNow();
  if (!now) return "--:--";
  time_t t = now;
  struct tm l;
  localtime_r(&t, &l);
  char b[8];
  strftime(b, sizeof(b), "%H:%M", &l);
  return b;
}

// ---------------------------------------------------------------- oznaczenia

static void tagsSave() { prefs.putBytes("tags", tags, sizeof(tags)); }

static void tagSet(const char *id, uint8_t st) {
  if (!id[0] || strlen(id) > 20) return;
  TagEntry *slot = nullptr;
  for (auto &t : tags) {
    if (t.status && !strcmp(t.id, id)) { slot = &t; break; }
    if (!t.status && !slot) slot = &t;
  }
  if (!slot) return;
  strlcpy(slot->id, id, sizeof(slot->id));
  slot->status = st;
}

static void tagRecordPending(const char *id, uint8_t st) {
  for (int i = 0; i < tagPendingCount; i++)
    if (!strcmp(tagPending[i].id, id)) { tagPending[i].status = st; return; }
  if (tagPendingCount < MAX_TAG_PENDING) {
    strlcpy(tagPending[tagPendingCount].id, id, sizeof(tagPending[0].id));
    tagPending[tagPendingCount++].status = st;
  }
}

static void printTags(bool pending) {
  Serial.print(pending ? "{\"tagpend\":{" : "{\"tags\":{");
  bool first = true;
  int count = pending ? tagPendingCount : MAX_TAGS;
  const TagEntry *list = pending ? tagPending : tags;
  for (int i = 0; i < count; i++) {
    if (!pending && !list[i].status) continue;
    const char *st = list[i].status == TAG_OWN ? "own" : list[i].status == TAG_FOREIGN ? "foreign" : "-";
    Serial.printf("%s\"%s\":\"%s\"", first ? "" : ",", list[i].id, st);
    first = false;
  }
  Serial.println("}}");
}

// ---------------------------------------------------------------- drony

static int activeDrones(DroneInfo **out) {
  uint32_t now = millis();
  int n = 0;
  for (auto &d : drones)
    if (d.used && now - d.lastMs < ACTIVE_MS) out[n++] = &d;
  for (int i = 1; i < n; i++)  // najsilniejszy sygnał pierwszy - najbliższy dron na górze
    for (int j = i; j > 0 && out[j]->rssi > out[j - 1]->rssi; j--) {
      DroneInfo *t = out[j];
      out[j] = out[j - 1];
      out[j - 1] = t;
    }
  return n;
}

static DroneInfo *selectedDrone(DroneInfo **list, int n) {
  for (int i = 0; i < n; i++)
    if (selId[0] && !strcmp(list[i]->id, selId)) { droneIdx = i; return list[i]; }
  if (!n) return nullptr;
  if (droneIdx >= n) droneIdx = 0;
  strlcpy(selId, list[droneIdx]->id, sizeof(selId));
  return list[droneIdx];
}

static uint16_t tagColor(uint8_t t) { return t == TAG_OWN ? TFT_GREEN : t == TAG_FOREIGN ? TFT_RED : TFT_ORANGE; }
static const char *tagName(uint8_t t) { return t == TAG_OWN ? "SWOJ" : t == TAG_FOREIGN ? "OBCY" : "NIEOZN."; }

// ---------------------------------------------------------------- dźwięk, dioda, ekran

static void led(bool on) { digitalWrite(PIN_LED, on ? HIGH : LOW); }

static void beep(uint16_t hz, uint16_t ms) {
  if (soundOn) M5.Speaker.tone(hz, ms);
}

static void wake() {
  lastInputMs = millis();
  if (!screenOn) {
    M5.Display.wakeup();
    screenOn = true;
  }
  if (dimmed || !screenOn) {
    M5.Display.setBrightness(BRIGHT_ON);
    dimmed = false;
  }
  dirty = true;
}

static void showToast(const char *t) {
  strlcpy(toast, t, sizeof(toast));
  toastUntil = millis() + 1500;
  dirty = true;
}

// ---------------------------------------------------------------- rysowanie (240 x 135)

static void drawHeader(const char *title, uint16_t bg) {
  auto &d = M5.Display;
  d.fillRect(0, 0, 240, 18, bg);
  d.setTextColor(TFT_WHITE, bg);
  d.setTextDatum(textdatum_t::middle_left);
  d.setFont(&fonts::Font2);
  d.drawString(title, 4, 9);
  d.setTextDatum(textdatum_t::middle_right);
  String r = clockText() + "  " + String(M5.Power.getBatteryLevel()) + "%" + (soundOn ? "" : "  CICHO") +
             (bleEnabled ? "" : "  WiFi") +
             (millis() - lastHostMs < 30000 ? "  PC" : "");
  d.drawString(r, 236, 9);
}

static void drawAlarm(DroneInfo **list, int n) {
  auto &d = M5.Display;
  int foreign = 0, own = 0;
  for (int i = 0; i < n; i++) (tagStatus(list[i]->id) == TAG_OWN ? own : foreign)++;
  uint16_t bg = foreign ? TFT_RED : own ? 0x0320 : TFT_BLACK;
  d.fillScreen(bg);
  drawHeader("ALARM", foreign ? 0x9000 : 0x2104);
  d.setTextColor(TFT_WHITE, bg);
  d.setTextDatum(textdatum_t::middle_center);
  d.setFont(&fonts::Font4);
  d.setTextSize(foreign ? 1.6f : 1.3f);
  d.drawString(foreign ? "DRON!" : own ? "SWOJ" : "BRAK", 120, 58);
  d.setTextSize(1);
  d.setFont(&fonts::Font2);
  if (foreign) d.drawString("w poblizu: " + String(foreign) + (own ? "  (+ swoje: " + String(own) + ")" : ""), 120, 92);
  else if (own) d.drawString("tylko swoje drony: " + String(own), 120, 92);
  else d.drawString((bleEnabled ? "nasluch Wi-Fi + BLE, CH" : "nasluch tylko Wi-Fi, CH") + String(currentChannel), 120, 92);
  if (n) {
    const DroneInfo &x = *list[0];  // najbliższy (najsilniejszy sygnał)
    String s = (x.id[0] ? String(x.id + max(0, (int)strlen(x.id) - 10)) : String("bez ID")) + "  " + String(x.rssi) + " dBm";
    if (x.hasOp && x.hasLoc) s += "  op " + String((int)distM(x.lat, x.lon, x.opLat, x.opLon)) + "m";
    if (x.wifiKind) s += "  Wi-Fi";
    d.drawString(s, 120, 118);
  }
}

static void drawDrones(DroneInfo **list, int n) {
  auto &d = M5.Display;
  d.fillScreen(TFT_BLACK);
  DroneInfo *x = selectedDrone(list, n);
  if (!x) {
    drawHeader("DRONY", 0x2104);
    d.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    d.setTextDatum(textdatum_t::middle_center);
    d.setFont(&fonts::Font2);
    d.drawString("brak dronow w poblizu", 120, 70);
    return;
  }
  uint8_t t = tagStatus(x->id);
  drawHeader(("DRON " + String(droneIdx + 1) + "/" + String(n) + "  " + tagName(t)).c_str(), tagColor(t) == TFT_GREEN ? 0x0320 : tagColor(t) == TFT_RED ? 0x9000 : 0x7A00);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setTextDatum(textdatum_t::top_left);
  d.setFont(&fonts::Font2);
  d.drawString(x->id[0] ? x->id : "bez ID", 4, 22);
  if (x->wifiKind) {  // dron Wi-Fi bez Remote ID: rodzaj zamiast wysokości i pozycji
    d.drawString(String("dron Wi-Fi: ") + x->wifiKind, 4, 42);
    d.drawString("kanal " + String(x->wifiCh) + ", brak Remote ID", 4, 62);
    d.drawString("RSSI " + String(x->rssi) + " dBm   " + String((millis() - x->lastMs) / 1000) + " s temu", 4, 82);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.drawString("B: nastepny  A przytrzymaj: oznacz", 4, 112);
    return;
  }
  String l2 = "wys " + (isnan(x->height) ? String("-") : String((int)x->height) + " m");
  if (!isnan(x->speed)) l2 += "   " + String(x->speed, 0) + " m/s";
  d.drawString(l2, 4, 42);
  d.drawString((x->hasOp && x->hasLoc) ? "operator " + String((int)distM(x->lat, x->lon, x->opLat, x->opLon)) + " m od drona"
                                       : String(x->hasLoc ? "operator: brak pozycji" : "brak pozycji drona"), 4, 62);
  d.drawString("RSSI " + String(x->rssi) + " dBm   " + String((millis() - x->lastMs) / 1000) + " s temu", 4, 82);
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  d.drawString("B: nastepny  A przytrzymaj: oznacz", 4, 112);
}

static void drawHunt(DroneInfo **list, int n) {
  auto &d = M5.Display;
  d.fillScreen(TFT_BLACK);
  DroneInfo *x = selectedDrone(list, n);
  drawHeader(x ? ("SZUKAJ " + String(x->id[0] ? x->id + max(0, (int)strlen(x->id) - 8) : "?")).c_str() : "SZUKAJ", 0x0015);
  d.setTextDatum(textdatum_t::middle_center);
  if (!x) {
    d.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    d.setFont(&fonts::Font2);
    d.drawString("brak dronow do szukania", 120, 70);
    return;
  }
  bool fresh = millis() - x->lastMs < 5000;
  d.setTextColor(fresh ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
  d.setFont(&fonts::Font4);
  d.setTextSize(1.5f);
  d.drawString(fresh ? String(x->rssi) + " dBm" : String("brak sygnalu"), 120, 52);
  d.setTextSize(1);
  int w = fresh ? constrain((x->rssi + 100) * 230 / 60, 0, 230) : 0;  // -100 ... -40 dBm
  d.fillRect(5, 84, 230, 14, 0x2104);
  d.fillRect(5, 84, w, 14, w > 150 ? TFT_RED : w > 75 ? TFT_ORANGE : TFT_GREEN);
  d.setFont(&fonts::Font2);
  d.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  d.drawString("max " + String(huntMax) + " dBm   B: inny dron", 120, 116);
}

static void drawSettings() {
  auto &d = M5.Display;
  d.fillScreen(TFT_BLACK);
  drawHeader("USTAWIENIA v" FW_VERSION, 0x2104);
  const String items[] = {
      String("Dzwiek: ") + (soundOn ? "wlaczony" : "wylaczony"),
      String("Bluetooth: ") + (bleEnabled ? "wl. (Wi-Fi + BLE)" : "wyl. (tylko Wi-Fi)"),
      String("Kanal Wi-Fi: ") + (fixedChannel ? "staly 6" : "skakanie"),
  };
  d.setFont(&fonts::Font2);
  d.setTextDatum(textdatum_t::middle_left);
  for (int i = 0; i < 3; i++) {
    uint16_t bg = i == setSel ? 0x0015 : TFT_BLACK;
    d.fillRect(0, 24 + i * 26, 240, 24, bg);
    d.setTextColor(TFT_WHITE, bg);
    d.drawString((i == setSel ? "> " : "  ") + items[i], 6, 36 + i * 26);
  }
  d.setTextColor(TFT_DARKGREY, TFT_BLACK);
  d.drawString("B: wybierz   B przytrzymaj: zmien", 6, 124);
}

static void render() {
  if (!screenOn) return;
  DroneInfo *list[MAX_DRONES];
  int n = activeDrones(list);
  if (view == V_ALARM) drawAlarm(list, n);
  else if (view == V_DRONES) drawDrones(list, n);
  else if (view == V_HUNT) drawHunt(list, n);
  else drawSettings();
  if ((int32_t)(millis() - toastUntil) < 0) {
    auto &d = M5.Display;
    d.fillRoundRect(20, 50, 200, 34, 6, TFT_NAVY);
    d.setTextColor(TFT_WHITE, TFT_NAVY);
    d.setTextDatum(textdatum_t::middle_center);
    d.setFont(&fonts::Font2);
    d.drawString(toast, 120, 67);
  }
}

// ---------------------------------------------------------------- USB (protokół jak Heltec)

static void printFrame(const Frame &f) {
  static const char hex[] = "0123456789abcdef";
  static char line[700];
  int n = snprintf(line, sizeof(line),
                   "{\"t\":\"%s\",\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"rssi\":%d,\"ch\":%u,\"dev\":\"m5\",\"d\":\"",
                   TRANSPORT_NAMES[f.transport], f.mac[0], f.mac[1], f.mac[2], f.mac[3], f.mac[4], f.mac[5], f.rssi,
                   f.channel);
  for (size_t i = 0; i < f.len && n < (int)sizeof(line) - 4; i++) {
    line[n++] = hex[f.data[i] >> 4];
    line[n++] = hex[f.data[i] & 0x0F];
  }
  line[n++] = '"';
  line[n++] = '}';
  line[n] = 0;
  Serial.println(line);
}

static void printStatus() {
  Serial.printf("{\"status\":{\"dev\":\"m5\",\"beacon\":%u,\"nan\":%u,\"ble\":%u,\"dropped\":%u,\"ch\":%u,"
                "\"all_beacon\":%u,\"all_ble\":%u,\"ble_on\":%s,\"bat_pct\":%d,\"bat_mv\":%d,\"boot\":%u,\"up\":%u,\"wifi_drone\":%u}}\n",
                counts[T_BEACON], counts[T_NAN], counts[T_BLE], dropped, currentChannel, allBeacon, allBle,
                bleEnabled ? "true" : "false", M5.Power.getBatteryLevel(), M5.Power.getBatteryVoltage(), bootId, millis() / 1000, wdSeen);
}

static void handleSerialCommand() {
  static char buf[64];
  static size_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (n < sizeof(buf) - 1) buf[n++] = c;
      continue;
    }
    buf[n] = 0;
    if (n) lastHostMs = millis();
    if (!strcasecmp(buf, "ID")) {
      Serial.println("{\"info\":\"RID sensor M5StickC Plus2\",\"fw\":\"" FW_VERSION "\",\"dev\":\"m5\"}");
      printStatus();
    } else if (!strncasecmp(buf, "TIME ", 5)) {
      uint32_t t = strtoul(buf + 5, nullptr, 10);
      if (t > 1700000000UL) setClock(t);
    } else if (!strncasecmp(buf, "TAG ", 4)) {
      char *st = strchr(buf + 4, ' ');
      if (st) {
        *st++ = 0;
        tagSet(buf + 4, !strcasecmp(st, "own") ? TAG_OWN : !strcasecmp(st, "foreign") ? TAG_FOREIGN : TAG_NONE);
        tagsSave();
        dirty = true;
      }
    } else if (!strcasecmp(buf, "TAGCLR")) {
      memset(tags, 0, sizeof(tags));
      tagsSave();
      dirty = true;
    } else if (!strcasecmp(buf, "TAGS")) {
      printTags(false);
    } else if (!strcasecmp(buf, "TAGPEND")) {
      printTags(true);
    } else if (!strcasecmp(buf, "TAGPENDCLR")) {
      tagPendingCount = 0;
    } else if (!strcasecmp(buf, "DUMP")) {  // brak pamięci zapisu na M5
      Serial.printf("{\"dump_begin\":{\"records\":0,\"boot\":%u,\"up\":%u,\"fs\":false,\"boots\":{}}}\n", bootId, millis() / 1000);
      Serial.println("{\"dump_end\":{\"records\":0}}");
    } else if (n >= 3 && !strncasecmp(buf, "CH", 2)) {
      int ch = atoi(buf + 2);
      if (ch >= 0 && ch <= 13) fixedChannel = ch;
    }
    n = 0;
  }
}

// ---------------------------------------------------------------- przyciski

static void handleButtons() {
  // włączanie to przytrzymanie przycisku zasilania: program startuje, zanim użytkownik go puści,
  // i wcześniej od razu uznawał to za "przytrzymaj = wyłącz". Przycisk działa dopiero po puszczeniu.
  static bool pwrReady = false;
  if (!pwrReady) {
    if (M5.BtnPWR.isPressed()) {
      M5.BtnPWR.wasHold();
      M5.BtnPWR.wasClicked();
      M5.BtnPWR.wasPressed();
    } else {
      pwrReady = true;
      M5.BtnPWR.wasClicked();
      M5.BtnPWR.wasHold();
      M5.BtnPWR.wasPressed();
    }
  }
  bool any = M5.BtnA.wasPressed() || M5.BtnB.wasPressed() || (pwrReady && M5.BtnPWR.wasPressed());
  static uint32_t wokeMs = 0;
  if (any && (!screenOn || dimmed)) {  // pierwsze naciśnięcie tylko budzi ekran
    wake();
    wokeMs = millis();  // puszczenie tego przycisku to nie "kliknięcie" (inaczej ekran od razu gaśnie)
    M5.BtnA.wasClicked();
    M5.BtnB.wasClicked();
    M5.BtnPWR.wasClicked();
    return;
  }
  if (any) lastInputMs = millis();
  DroneInfo *list[MAX_DRONES];
  int n = activeDrones(list);
  bool woke = millis() - wokeMs < 2500;  // A/B, które obudziły ekran, nie zmieniają widoku
  if (M5.BtnA.wasClicked() && !woke) {
    view = view == V_ALARM ? V_DRONES : view == V_DRONES ? V_HUNT : view == V_HUNT ? V_SETTINGS : V_ALARM;
    huntMax = -127;
    dirty = true;
  }
  if (M5.BtnA.wasHold() && view == V_DRONES) {
    DroneInfo *x = selectedDrone(list, n);
    if (x && x->id[0]) {
      uint8_t st = tagStatus(x->id);
      st = st == TAG_NONE ? TAG_OWN : st == TAG_OWN ? TAG_FOREIGN : TAG_NONE;
      tagSet(x->id, st);
      tagsSave();
      tagRecordPending(x->id, st);  // trafi na stronę RID Decoder przy podłączeniu do agenta
      showToast(st == TAG_OWN ? "Oznaczono: SWOJ" : st == TAG_FOREIGN ? "Oznaczono: OBCY" : "Usunieto oznaczenie");
    } else {
      showToast("Dron bez ID");
    }
  }
  if (M5.BtnB.wasClicked() && !woke) {
    if (view == V_SETTINGS) {
      setSel = (setSel + 1) % 3;
    } else if (view == V_ALARM) {
      view = V_DRONES;
    } else if (n) {
      droneIdx = (droneIdx + 1) % n;
      strlcpy(selId, list[droneIdx]->id, sizeof(selId));
      huntMax = -127;
    }
    dirty = true;
  }
  if (M5.BtnB.wasHold()) {
    if (view == V_SETTINGS && setSel == 1) {
      bleSetEnabled(!bleEnabled);
      prefs.putBool("ble", bleEnabled);
      showToast(bleEnabled ? "Wi-Fi + Bluetooth" : "Tylko Wi-Fi");
    } else if (view == V_SETTINGS && setSel == 2) {
      fixedChannel = fixedChannel ? 0 : 6;
      showToast(fixedChannel ? "Staly kanal 6" : "Skakanie po kanalach");
    } else {
      soundOn = !soundOn;
      prefs.putBool("sound", soundOn);
      showToast(soundOn ? "Dzwiek wlaczony" : "Dzwiek wylaczony");
    }
  }
  bool pwrClick = M5.BtnPWR.wasClicked();
  if (pwrReady && pwrClick && !woke) {
    screenOn = false;
    M5.Display.sleep();
  }
  if (pwrReady && M5.BtnPWR.wasHold()) {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Power.powerOff();
  }
}

// ---------------------------------------------------------------- start i pętla

// nowy dron (Remote ID albo Wi-Fi): alarm, gdy nie jest oznaczony jako swój
static void newDrone(DroneInfo *d) {
  d->shown = true;
  if (tagStatus(d->id) != TAG_OWN) {
    wake();
    view = V_ALARM;
    for (int i = 0; i < 3; i++) { beep(2200, 90); delay(130); }
    led(true);
    ledUntil = millis() + 5000;
    lastRemindMs = millis();
  }
  dirty = true;
}

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 921600;
  M5.begin(cfg);  // M5Unified: ekran, przyciski, brzęczyk, RTC, podtrzymanie zasilania (GPIO4)
  M5.BtnPWR.setHoldThresh(2000);  // wyłączenie: przytrzymanie przycisku zasilania 2 s
  setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
  tzset();
  pinMode(PIN_LED, OUTPUT);
  led(false);
  M5.Display.setRotation(1);
  M5.Display.setBrightness(BRIGHT_ON);
  M5.Speaker.setVolume(200);

  prefs.begin("ridm5", false);
  bootId = prefs.getUShort("boot", 0) + 1;
  prefs.putUShort("boot", bootId);
  soundOn = prefs.getBool("sound", true);
  bleEnabled = prefs.getBool("ble", true);
  if (prefs.getBytes("tags", tags, sizeof(tags)) != sizeof(tags)) memset(tags, 0, sizeof(tags));
  uint32_t rtc = unixFromRtc();
  if (rtc) unixBase = rtc - millis() / 1000;

  frameQueue = xQueueCreate(32, sizeof(Frame));
  radioBegin();
  lastInputMs = millis();
  Serial.println("{\"info\":\"RID sensor M5StickC Plus2 gotowy\",\"dev\":\"m5\"}");
}

void loop() {
  static uint32_t lastDraw = 0, lastStatus = 0;
  M5.update();
  Frame f;
  while (xQueueReceive(frameQueue, &f, pdMS_TO_TICKS(5)) == pdTRUE) {
    printFrame(f);
    DroneInfo *d = parseFrame(f);
    if (d && !d->shown && (d->id[0] || millis() - d->firstMs > 5000)) newDrone(d);  // chwila na Basic ID
  }
  WdEvent we;
  while (xQueueReceive(wdQueue, &we, 0) == pdTRUE) {  // dron Wi-Fi bez Remote ID
    DroneInfo *d = wdApply(we);
    if (!d->shown) newDrone(d);
    wdPrint(Serial, *d);
  }
  handleSerialCommand();
  handleButtons();

  uint32_t now = millis();
  DroneInfo *list[MAX_DRONES];
  int n = activeDrones(list);
  int foreign = 0;
  for (int i = 0; i < n; i++) foreign += tagStatus(list[i]->id) != TAG_OWN;
  if (foreign && now - lastRemindMs > REMIND_MS) {  // obcy dron nadal w pobliżu
    lastRemindMs = now;
    beep(2200, 120);
  }
  if (ledUntil) {  // miganie diodą przy nowym obcym dronie
    led((now / 250) % 2);
    if (now > ledUntil) { ledUntil = 0; led(false); }
  }
  if (view == V_HUNT) {  // "licznik Geigera": im silniejszy sygnał, tym częściej
    DroneInfo *x = selectedDrone(list, n);
    if (x && now - x->lastMs < 5000) {
      if (x->rssi > huntMax) huntMax = x->rssi;
      uint32_t interval = map(constrain(x->rssi, -100, -40), -100, -40, 1500, 120);
      if (now - lastHuntBeep > interval) {
        lastHuntBeep = now;
        beep(1500 + (x->rssi + 100) * 15, 30);
      }
    }
  }
  if (screenOn && !dimmed && now - lastInputMs > DIM_AFTER_MS && view != V_HUNT) {
    M5.Display.setBrightness(BRIGHT_DIM);
    dimmed = true;
  }
  if (screenOn && now - lastInputMs > SLEEP_AFTER_MS && !foreign && view != V_HUNT) {
    M5.Display.sleep();
    screenOn = false;
  }
  if (dirty || now - lastDraw > 1000) {
    dirty = false;
    lastDraw = now;
    render();
  }
  if (now - lastStatus > 10000) {
    lastStatus = now;
    printStatus();
  }
}
