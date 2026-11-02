# RID M5: osobisty alarm Remote ID na M5StickC Plus2 (wersja 1.6)

Firmware dla **M5StickC Plus2**. Zamienia go w mały wykrywacz dronów do noszenia przy sobie:

- Odbiera Remote ID przez **Wi-Fi Beacon**, **Wi-Fi NAN** i **BLE 4**. BLE 5 Long Range nie działa.
- **Piszczy przy nowym dronie, który nie jest oznaczony jako swój.**
- Ma tryb **SZUKAJ**, który piszczy tym szybciej, im silniejszy jest sygnał drona, więc prowadzi do drona lub operatora.
- Wykrywa też **drony Wi-Fi bez Remote ID** (DJI Tello, Parrot, zabawkowe drony z kamerą Wi-Fi).
- Wbudowany zegar RTC pamięta godzinę także bez komputera.
- Przez USB-C działa jako sensor agenta RID Decoder na komputerze (opcjonalnie).
  Oznaczenia swój/obcy są wtedy wspólne z Heltec, CYD i stroną RID Decoder. Strona dostaje też zajętość
  kanałów Wi-Fi 2,4 GHz (panel pasma).

Ten sam numer wersji mają firmware dla [Dron Detector (Heltec V3)](https://github.com/QLDADM/dron-detector) i [CYD](https://github.com/QLDADM/rid-cyd).

---

## Wgranie krok po kroku (bez instalowania programów)

### Co jest potrzebne

- **M5StickC Plus2** (wersja „Plus2”; starsze M5StickC i M5StickC Plus mają inny układ zasilania).
- **Kabel USB-C do transmisji danych.** Część kabli służy tylko do ładowania i z nimi komputer nie widzi urządzenia.
- Komputer z przeglądarką **Google Chrome** albo **Microsoft Edge**. Firefox i Safari nie obsługują tej metody.

### Krok 1. Pobierz plik firmware

1. Wejdź na https://github.com/QLDADM/rid-m5/releases.
2. Przy najnowszej wersji (np. **v1.6**) w sekcji **Assets** kliknij **`rid-m5stickc-plus2-full.bin`**.
3. Plik trafi do folderu **Pobrane**. Nie otwieraj go i nie zmieniaj mu nazwy.

### Krok 2. Podłącz urządzenie

1. Podłącz M5StickC Plus2 kablem USB-C do komputera.
2. Zamknij programy, które mogą używać portu: agenta RID Decoder, M5Burner, Arduino, monitor portu.
3. **Windows:** M5StickC Plus2 ma układ USB **CH9102**. Jeśli w kroku 4 przeglądarka nie pokazuje żadnego portu,
   zainstaluj sterownik ze strony producenta: https://www.wch-ic.com/downloads/CH343SER_EXE.html.
   Potem odłącz i podłącz urządzenie ponownie.

### Krok 3. Otwórz programator w przeglądarce

1. W Chrome albo Edge wejdź na **https://espressif.github.io/esptool-js/**.
2. Zostaw **puste** pole wyboru **WebUSB (CH340)** oraz **Show Debug log**.
3. W sekcji **Program** zostaw **Baudrate: 921600**.

### Krok 4. Połącz się z urządzeniem

1. Kliknij **Connect**.
2. Wybierz port podobny do **„USB-Enhanced-SERIAL CH9102 (COM6)”**. Numer COM może być inny. Kliknij **Połącz**.
3. Po kilku sekundach w konsoli na dole pojawi się **„ESP32”**, a w sekcji Program **pojawi się tabela z polem
   „Flash Address”**. Przed połączeniem tej tabeli nie ma.

Jeśli połączenie się nie udaje, odłącz kabel, włącz urządzenie przyciskiem zasilania (lewy bok), podłącz kabel
ponownie i kliknij **Connect** jeszcze raz.

### Krok 5. Wgraj firmware

1. W polu **Flash Address** wpisz **`0x0`** (zero, iks, zero).
2. Kliknij **Wybierz plik** w tym samym wierszu i wskaż pobrany **`rid-m5stickc-plus2-full.bin`**.
3. Kliknij **Program** i poczekaj, aż pasek dojdzie do 100% (około 30–60 s). **Nie odłączaj kabla w trakcie.**
4. Odłącz kabel albo przytrzymaj przycisk zasilania, żeby wyłączyć, a potem włącz urządzenie ponownie.

Gotowe. Na ekranie pojawi się widok ALARM. Wgranie pliku `full.bin` kasuje ustawienia (dźwięk, Bluetooth, oznaczenia).

---

## Przyciski

| Przycisk | Działanie |
|---|---|
| **A** (przód) | następny widok: ALARM → DRONY → SZUKAJ → USTAWIENIA |
| **B** (bok) | następny dron (DRONY / SZUKAJ); na ALARM otwiera listę dronów |
| **A** przytrzymaj | w DRONY: oznaczenie brak → SWOJ → OBCY |
| **B** przytrzymaj | w USTAWIENIA: zmiana zaznaczonej opcji, w pozostałych widokach dźwięk wł./wył. |
| zasilanie krótko | ekran wł./wył. |
| zasilanie przytrzymaj 2 s | wyłączenie |
| zasilanie przytrzymaj ok. 2 s (wyłączony) | włączenie |

Pierwsze naciśnięcie przy przyciemnionym albo wygaszonym ekranie tylko go budzi.

## Widoki

- **ALARM**: czerwone tło i **DRON!** przy obcym lub nieoznaczonym dronie, zielone **SWOJ**, gdy w pobliżu są tylko
  swoje, a **BRAK**, gdy nic nie lata. Na dole najbliższy dron (najsilniejszy sygnał), RSSI i odległość do operatora.
- **DRONY**: drony po kolei, od najbliższego. Widać ID, klasę UE, oznaczenie, wysokość, prędkość poziomą i pionową, numer operatora, odległość dron–operator,
  RSSI i wiek danych. Przy dronie Wi-Fi bez Remote ID widać „dron Wi-Fi: rodzaj” i kanał zamiast wysokości.
- **SZUKAJ**: duży odczyt siły sygnału wybranego drona, pasek i maksimum. **Piszczy tym szybciej, im silniejszy sygnał**,
  więc idąc w stronę szybszego piszczenia zbliżasz się do drona.
- **USTAWIENIA** (B wybiera, przytrzymanie B zmienia): dźwięk; **Bluetooth** wł./wył. (tryb „tylko Wi-Fi” daje całe
  radio dla Wi-Fi i trochę oszczędza baterię, ale nie widać wtedy dronów nadających tylko przez BLE); kanał Wi-Fi
  (skakanie albo stały 6). W nagłówku widać „WiFi”, gdy BLE jest wyłączone.

Nowy obcy dron daje trzy piski, miganie diody i ekran ALARM. Gdy nadal jest w pobliżu, przypomnienie pada co minutę.
Ekran przyciemnia się po 20 s i gaśnie po minucie bez obcych dronów, żeby oszczędzać baterię (około 1,5–2 h nasłuchu).

---

## Dla zaawansowanych

### Wgranie z wiersza poleceń

```powershell
pip install esptool
esptool --chip esp32 --baud 921600 write_flash 0x0 rid-m5stickc-plus2-full.bin
```

### Budowanie ze źródeł

```powershell
pip install platformio
pio run -t upload
```

Wgranie przez PlatformIO nie kasuje ustawień.

### Protokół USB (921600 bod)

Linie JSON jak w Heltec, z polem `"dev":"m5"`: ramki `{"t":...}`, status, drony Wi-Fi `{"wd":{...}}`,
a co ok. 10 s nasłuchu zajętość kanałów Wi-Fi 1–13: `{"cs":{"b":"2G4","r":[ramek/s...],"p":[najsilniejszy dBm...]}}`
(`-128` = brak sygnału na kanale).
Polecenia: `ID`, `CH <n>`, `TIME <unix>`, `TAG <id> own|foreign|-`, `TAGS`, `TAGCLR`, `TAGPEND`, `PING`.
