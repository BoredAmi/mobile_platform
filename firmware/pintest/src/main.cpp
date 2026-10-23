/*
 * Wiring test for the jgb_rover Teensy 4.1 board (no motors: the driver enable pin is held low).
 *
 * Every 0.5 s: level and edge count of each encoder pin. Turn a wheel by hand: both A and B of
 * that wheel must count edges, about the same number. A pin that stays at 1 with 0 edges gets no
 * signal (wire, connector, encoder power); a pin stuck at 0 is shorted to GND or the encoder pulls
 * it down without power.
 * Every 3 s: I2C scan on all three buses. The GY-521 must show up as 0x68 on Wire2 (SDA 25, SCL 24).
 * Idle SDA / SCL levels are printed first: both must be 1 (the GY-521 pull-ups); 0 = no power on
 * the GY-521 or a short.
 * Pin numbers are copied from firmware/teensy41/src/config.h.
 */
#include <Arduino.h>
#include <Wire.h>

struct EncPin {
  const char *name;
  uint8_t pin;
  volatile uint32_t edges;
};

static EncPin pins[] = {
    {"left A (20)", 20, 0},
    {"left B (3)", 3, 0},
    {"right A (4)", 4, 0},
    {"right B (5)", 5, 0},
};

static void isr0() { pins[0].edges++; }
static void isr1() { pins[1].edges++; }
static void isr2() { pins[2].edges++; }
static void isr3() { pins[3].edges++; }
static void (*const isrs[])() = {isr0, isr1, isr2, isr3};

struct Bus {
  const char *name;
  TwoWire *wire;
  uint8_t sda, scl;
};

static const Bus buses[] = {
    {"Wire  (SDA 18, SCL 19)", &Wire, 18, 19},
    {"Wire1 (SDA 17, SCL 16)", &Wire1, 17, 16},
    {"Wire2 (SDA 25, SCL 24)", &Wire2, 25, 24},
};

static void scan(const Bus &b) {
  b.wire->end();
  pinMode(b.sda, INPUT_PULLUP);
  pinMode(b.scl, INPUT_PULLUP);
  delayMicroseconds(50);
  int sda = digitalRead(b.sda), scl = digitalRead(b.scl);
  b.wire->begin();
  b.wire->setClock(100000);
  Serial.printf("  %s: idle SDA %d SCL %d, found:", b.name, sda, scl);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    b.wire->beginTransmission(a);
    if (b.wire->endTransmission() == 0) {
      Serial.printf(" 0x%02X%s", a, a == 0x68 || a == 0x69 ? " (MPU6050)" : "");
      ++found;
    }
  }
  Serial.println(found ? "" : " nothing");
}

/* Software I2C probe on any pin pair, to find a GY-521 wired to the wrong pins. Open drain: a pin
 * is either driven low or released to its pull-up. Motor pins 6-10 and the encoder pins are never
 * touched. */
static const uint8_t search_pins[] = {0, 1, 11, 12, 14, 15, 16, 17, 18, 19, 21, 22, 23, 24, 25, 26, 27};

static void od(uint8_t pin, bool high) {
  if (high) {
    pinMode(pin, INPUT_PULLUP);
  } else {
    pinMode(pin, OUTPUT);
    digitalWriteFast(pin, LOW);
  }
  delayMicroseconds(5);
}

static bool sw_probe(uint8_t sda, uint8_t scl, uint8_t addr) {
  od(sda, true);
  od(scl, true);
  if (!digitalRead(sda) || !digitalRead(scl)) return false;
  od(sda, false);  // START
  od(scl, false);
  uint8_t byte = addr << 1;
  for (int i = 7; i >= 0; --i) {
    od(sda, (byte >> i) & 1);
    od(scl, true);
    od(scl, false);
  }
  od(sda, true);  // ACK bit: the device pulls SDA low
  od(scl, true);
  bool ack = !digitalRead(sda);
  od(scl, false);
  od(sda, false);  // STOP
  od(scl, true);
  od(sda, true);
  return ack;
}

/* With the internal ~100k pull-down, only a pin with an external pull-up (GY-521 SDA / SCL, an
 * encoder output) or a driven signal reads 1; an unconnected pin reads 0. */
static void pullup_search() {
  uint32_t edges[4];
  for (int i = 0; i < 4; ++i) edges[i] = pins[i].edges;
  Serial.print("pins pulled high externally:");
  for (uint8_t p = 0; p <= 41; ++p) {
    if (p >= 6 && p <= 10) continue;  // motor driver pins stay outputs
    if (p == 13) continue;            // LED
    pinMode(p, INPUT_PULLDOWN);
    delayMicroseconds(200);
    if (digitalRead(p)) Serial.printf(" %d", p);
  }
  Serial.println();
  for (const EncPin &e : pins) pinMode(e.pin, INPUT_PULLUP);
  delayMicroseconds(200);
  for (int i = 0; i < 4; ++i) pins[i].edges = edges[i];  // switching the pulls is not wheel motion
}

/* SDA / SCL on their own, with the I2C hardware off. 'pulldown' = 1 means something outside pulls
 * the pin up harder than the internal ~100k pull-down; 'released' = 1 means that, after the pin was
 * driven low for 1 ms, it came back high by itself (an external pull-up; a floating pin stays 0). */
static void i2c_pin_report() {
  for (const Bus &b : buses) b.wire->end();
  for (uint8_t p : {18, 19}) {
    pinMode(p, INPUT);
    delayMicroseconds(50);
    int floating = digitalRead(p);
    pinMode(p, INPUT_PULLDOWN);
    delayMicroseconds(200);
    int pulldown = digitalRead(p);
    pinMode(p, OUTPUT);
    digitalWriteFast(p, LOW);
    delay(1);
    pinMode(p, INPUT);
    delayMicroseconds(30);
    int released = digitalRead(p);
    Serial.printf("  pin %d (%s): floating %d, with pull-down %d, released after low %d\n", p,
                  p == 18 ? "SDA" : "SCL", floating, pulldown, released);
  }
}

static void pin_search() {
  i2c_pin_report();
  pullup_search();
  for (const Bus &b : buses) b.wire->end();
  Serial.print("software I2C search for 0x68 / 0x69 on all pin pairs:");
  int found = 0;
  for (uint8_t sda : search_pins) {
    for (uint8_t scl : search_pins) {
      if (sda == scl) continue;
      for (uint8_t a : {0x68, 0x69}) {
        if (sw_probe(sda, scl, a)) {
          Serial.printf(" [0x%02X with SDA on pin %d, SCL on pin %d]", a, sda, scl);
          ++found;
        }
      }
    }
  }
  for (uint8_t p : search_pins) pinMode(p, INPUT);
  Serial.println(found ? "" : " nothing");
}

void setup() {
  pinMode(10, OUTPUT);  // BTS7960 R_EN / L_EN: low = bridges off
  digitalWriteFast(10, LOW);
  for (uint8_t p : {6, 7, 8, 9}) {
    pinMode(p, OUTPUT);
    digitalWriteFast(p, LOW);
  }
  for (int i = 0; i < 4; ++i) {
    pinMode(pins[i].pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pins[i].pin), isrs[i], CHANGE);
  }
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println("jgb_rover pin test: motors off. Turn each wheel by hand.");
}

void loop() {
  static uint32_t next_scan;
  if ((int32_t)(millis() - next_scan) >= 0) {
    pin_search();
    Serial.println("I2C scan:");
    for (const Bus &b : buses) scan(b);
    next_scan = millis() + 3000;
  }
  for (const EncPin &p : pins) {
    Serial.printf("%s=%d edges %-6lu  ", p.name, digitalReadFast(p.pin), (unsigned long)p.edges);
  }
  Serial.println();
  delay(500);
}
