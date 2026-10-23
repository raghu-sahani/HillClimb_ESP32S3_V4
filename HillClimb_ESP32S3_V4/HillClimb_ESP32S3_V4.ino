/*
  HILL CLIMB MINI 4.0.1 - REAL ROLLOVERS + full-frame ESP32-S3 game
  ============================================================
  TFT ST7735 128x160, rotated to 160x128 landscape:
    LED  -> 3V3     SCK -> GPIO12     SDA/MOSI -> GPIO11
    A0/DC-> GPIO9   RST -> GPIO10     CS -> GPIO8
    VCC  -> 3V3     GND -> GND

  LEFT / BRAKE: GPIO4 -> momentary button -> GND
  RIGHT / GAS:  GPIO5 -> momentary button -> GND
  PASSIVE 3.3V PIEZO: + -> GPIO6, - -> GND

  Libraries: Adafruit GFX Library; Adafruit ST7735 and ST7789 Library
  Arduino board: ESP32S3 Dev Module (esp32 by Espressif Systems)

  Flicker fix: render ENTIRE frame to RAM using GFXcanvas16, then
  send ONE completed 160 x 128 RGB565 image per frame to the TFT.
  Never clear the physical TFT inside the animation loop.

  This is a stylized 2D mini racing game, not the original copyrighted game.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <string.h>

// GPIO connections - SAME as your existing wiring
constexpr int TFT_CS = 8;
constexpr int TFT_DC = 9;
constexpr int TFT_RST = 10;
constexpr int TFT_MOSI = 11;
constexpr int TFT_SCK = 12;
constexpr int BRAKE_PIN = 4;
constexpr int GAS_PIN = 5;
constexpr int BUZZER_PIN = 6;

constexpr int W = 160;
constexpr int H = 128;
constexpr uint32_t FRAME_MS = 40;  // 25 FPS rendering to limit TFT traffic
constexpr uint32_t PHYSICS_MS = 10; // fixed 100 Hz physics, independent of screen refresh
constexpr uint32_t TFT_SPI_HZ = 24000000; // reliable starting point; try 16000000 if glitches
constexpr float PI_F = 3.14159265f;

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(W, H); // 40,960 bytes RAM on ESP32-S3

uint16_t C_SKY1, C_SKY2, C_SKY3, C_SUN, C_CLOUD;
uint16_t C_MOUNT1, C_MOUNT2, C_DIRT, C_DIRT2, C_SOIL;
uint16_t C_GRASS, C_GRASS2, C_ROCK, C_WHITE, C_DARK, C_GRAY;
uint16_t C_RED, C_RED_DARK, C_GLASS, C_GLASS2, C_GOLD, C_HIGHLIGHT;
uint16_t C_GREEN, C_FUEL, C_WHEEL, C_RIM, C_SILVER;

constexpr int COIN_MAX = 240;
constexpr int CAN_MAX  = 48;
bool coinTaken[COIN_MAX];
bool canTaken[CAN_MAX];

enum RunMode { MODE_TITLE, MODE_DRIVE, MODE_OVER };
RunMode mode = MODE_TITLE;

float carX = 36.0f;
float carY = 78.0f;       // wheel-axle height in world coordinates
float carSpeed = 0.0f;   // pixels per second
float fallSpeed = 0.0f;  // positive is down
float carTilt = 0.0f;    // radians; clockwise is positive
float tireSpin = 0.0f;
float tireSpinSpeed = 0.0f; // wheel angular velocity, can freewheel in the air
float suspension = 0.0f;      // visual spring compression (pixels)
float suspensionSpeed = 0.0f; // compression velocity
float pitchSpeed = 0.0f;      // chassis angular velocity (radians/sec)
float flightTime = 0.0f;      // seconds since takeoff
float takeoffAngle = 0.0f;    // continuous rotation reference for air flips
int flipsThisJump = 0;
int totalFlips = 0;
uint32_t flipTextUntil = 0;
float cameraX = 0.0f;
float fuel = 100.0f;
float distanceBest = 0.0f;
bool inAir = false;
bool gasBefore = false;
int score = 0;
const char *endReason = "OUT OF FUEL";

uint32_t lastFrame = 0;
uint32_t lastPhysics = 0;
uint32_t buzzerUntil = 0;
uint32_t lastMotorSound = 0;
uint32_t lastLowFuelSound = 0;
uint32_t lastSerialStatus = 0;

// ---- Basic sound compatible with ESP32 Arduino core 2.x AND 3.x ----
void buzzerHz(uint32_t hz) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(BUZZER_PIN, hz);
#else
  ledcWriteTone(0, hz);
#endif
}

void chirp(uint32_t hz, uint16_t durationMs) {
  buzzerHz(hz);
  buzzerUntil = millis() + durationMs;
}

void serviceSound(uint32_t now) {
  if (buzzerUntil && (int32_t)(now - buzzerUntil) >= 0) {
    buzzerHz(0);
    buzzerUntil = 0;
  }
}

// ---- Smooth, continuous physics terrain ----
float groundY(float x) {
  return 88.0f
       + 12.5f * sinf(x * 0.0150f)
       +  8.0f * sinf(x * 0.0340f + 0.4f)
       +  2.9f * sinf(x * 0.0870f)
       +  4.0f * sinf(x * 0.0055f + 0.8f);
}

float roadTilt(float x) {
  return atan2f(groundY(x + 13.0f) - groundY(x - 13.0f), 26.0f);
}

float axleGround(float x) {
  return 0.5f * (groundY(x - 13.0f) + groundY(x + 13.0f)) - 6.0f;
}

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Shortest signed angle to the road: handles rotations beyond 360 degrees.
// IMPORTANT: carTilt itself is never clamped or wrapped in the air, so a
// genuine 360-degree flip is physically possible and is counted correctly.
float wrapAngle(float a) {
  return atan2f(sinf(a), cosf(a));
}

// Test a point on the rotated car against the actual local terrain.
// The roof is checked separately from the wheels to detect upside-down crashes.
float contactDepth(float localX, float localY) {
  float cosine = cosf(carTilt);
  float sine = sinf(carTilt);
  float x = carX + localX * cosine - localY * sine;
  float y = carY + localX * sine + localY * cosine;
  return y - groundY(x);
}

int positiveMod(int x, int divisor) {
  int r = x % divisor;
  return r < 0 ? r + divisor : r;
}

void rotateXY(float cx, float cy, float angle, float dx, float dy,
              int16_t &ox, int16_t &oy) {
  float cc = cosf(angle), ss = sinf(angle);
  ox = (int16_t)lroundf(cx + dx * cc - dy * ss);
  oy = (int16_t)lroundf(cy + dx * ss + dy * cc);
}

void rotatedQuad(float cx, float cy, float angle,
                 float left, float top, float right, float bottom, uint16_t color) {
  int16_t x0, y0, x1, y1, x2, y2, x3, y3;
  rotateXY(cx, cy, angle, left, top, x0, y0);
  rotateXY(cx, cy, angle, right, top, x1, y1);
  rotateXY(cx, cy, angle, right, bottom, x2, y2);
  rotateXY(cx, cy, angle, left, bottom, x3, y3);
  canvas.fillTriangle(x0, y0, x1, y1, x2, y2, color);
  canvas.fillTriangle(x0, y0, x2, y2, x3, y3, color);
}

void rotatedLine(float cx, float cy, float angle,
                 float ax, float ay, float bx, float by, uint16_t color) {
  int16_t x0, y0, x1, y1;
  rotateXY(cx, cy, angle, ax, ay, x0, y0);
  rotateXY(cx, cy, angle, bx, by, x1, y1);
  canvas.drawLine(x0, y0, x1, y1, color);
}

// ---- Scrolling scenery ----
void drawCloud(int x, int y) {
  canvas.fillRoundRect(x - 14, y, 28, 7, 3, C_CLOUD);
  canvas.fillCircle(x - 6, y, 5, C_CLOUD);
  canvas.fillCircle(x + 3, y - 3, 7, C_CLOUD);
  canvas.fillCircle(x + 10, y, 4, C_CLOUD);
}

void drawBackdrop(float cam) {
  canvas.fillScreen(C_SKY1);
  canvas.fillRect(0, 22, W, 24, C_SKY2);
  canvas.fillRect(0, 46, W, 82, C_SKY3);

  // Sun: warm halo and bright center
  int sunX = 126 - positiveMod((int)(cam * 0.018f), 215);
  canvas.fillCircle(sunX, 36, 14, C_SKY2);
  canvas.fillCircle(sunX, 36, 10, C_SUN);
  canvas.fillCircle(sunX - 3, 33, 3, C_HIGHLIGHT);

  // Slow-moving white clouds
  int cloudOffset = positiveMod((int)(cam * 0.070f), 212);
  for (int i = 0; i < 4; ++i) {
    int x = i * 71 - cloudOffset + 18;
    drawCloud(x, 21 + (i % 2) * 14);
  }

  // Two parallax mountain ridges, rendered behind the foreground road
  int farOffset = positiveMod((int)(cam * 0.085f), 100);
  for (int i = -1; i < 4; ++i) {
    int x = i * 100 - farOffset;
    canvas.fillTriangle(x, 104, x + 48, 50, x + 100, 104, C_MOUNT1);
    canvas.fillTriangle(x + 40, 71, x + 48, 50, x + 59, 69, C_CLOUD);
  }
  int nearOffset = positiveMod((int)(cam * 0.18f), 87);
  for (int i = -1; i < 5; ++i) {
    int x = i * 87 - nearOffset;
    canvas.fillTriangle(x, 115, x + 42, 74, x + 87, 115, C_MOUNT2);
  }
}

void drawTrack(float cam) {
  // Two-pixel vertical strips, drawn ONLY to the off-screen canvas
  for (int sx = 0; sx < W; sx += 2) {
    float wx = cam + sx;
    int gy = (int)lroundf(groundY(wx));
    gy = constrain(gy, 22, H - 1);

    canvas.fillRect(sx, gy, 2, H - gy, C_DIRT);
    if (gy + 7 < H) canvas.fillRect(sx, gy + 7, 2, H - gy - 7, C_DIRT2);
    if (gy + 23 < H) canvas.fillRect(sx, gy + 23, 2, H - gy - 23, C_SOIL);
    canvas.drawFastHLine(sx, gy, 2, C_GRASS);
    if (gy + 1 < H) canvas.drawFastHLine(sx, gy + 1, 2, C_GRASS2);

    // Soil pebbles stay locked to the world instead of flickering randomly
    int tile = (int)floorf(wx / 13.0f);
    if (positiveMod(tile * 17 + 7, 7) < 2 && gy + 11 < H) {
      canvas.drawPixel(sx, gy + 9, C_ROCK);
    }
  }

  // Grass, rocks and little bushes: stable world-based decoration
  int start = (int)floorf(cam / 48.0f) - 1;
  for (int i = start; i < start + 7; ++i) {
    float wx = (float)i * 48.0f + 18.0f;
    int sx = (int)lroundf(wx - cam);
    int gy = (int)lroundf(groundY(wx));
    if (sx < -10 || sx > W + 10) continue;
    if (positiveMod(i * 31, 6) < 2) {
      canvas.fillCircle(sx, gy - 2, 3, C_GRASS2);
      canvas.fillCircle(sx + 3, gy - 2, 3, C_GRASS);
      canvas.drawFastVLine(sx + 1, gy - 7, 5, C_GRASS);
    } else if (positiveMod(i, 5) == 0) {
      canvas.fillTriangle(sx - 3, gy, sx, gy - 4, sx + 5, gy, C_ROCK);
    } else {
      canvas.drawLine(sx, gy, sx - 1, gy - 4, C_GRASS2);
      canvas.drawLine(sx, gy, sx + 2, gy - 3, C_GRASS);
    }
  }
}

// ---- Coin / fuel visuals and collision ----
float coinPosition(int i) { return 115.0f + 88.0f * i; }
float canPosition(int i) { return 305.0f + 475.0f * i; }

void drawItems(float cam, uint32_t now) {
  float left = cam - 12.0f, right = cam + W + 12.0f;
  int anim = (int)(now / 150);

  int firstCoin = (int)floorf((left - 115.0f) / 88.0f);
  if (firstCoin < 0) firstCoin = 0;
  for (int i = firstCoin; i < COIN_MAX; ++i) {
    float wx = coinPosition(i);
    if (wx > right) break;
    if (coinTaken[i]) continue;
    int sx = (int)lroundf(wx - cam);
    int cy = (int)lroundf(groundY(wx) - 28.0f + sinf(now * 0.006f + i) * 1.7f);
    int coinWidth = 2 + positiveMod(anim + i, 4);
    canvas.fillRoundRect(sx - coinWidth, cy - 6, coinWidth * 2 + 1, 12, 2, C_GOLD);
    canvas.drawRoundRect(sx - coinWidth, cy - 6, coinWidth * 2 + 1, 12, 2, C_HIGHLIGHT);
    if (coinWidth >= 4) canvas.drawFastVLine(sx, cy - 3, 7, C_RED_DARK);
  }

  int firstCan = (int)floorf((left - 305.0f) / 475.0f);
  if (firstCan < 0) firstCan = 0;
  for (int i = firstCan; i < CAN_MAX; ++i) {
    float wx = canPosition(i);
    if (wx > right) break;
    if (canTaken[i]) continue;
    int sx = (int)lroundf(wx - cam);
    int y = (int)lroundf(groundY(wx) - 29);
    canvas.fillRoundRect(sx - 6, y - 6, 12, 14, 2, C_RED_DARK);
    canvas.fillRect(sx - 5, y - 4, 10, 10, C_FUEL);
    canvas.fillRect(sx - 2, y - 9, 6, 3, C_DARK);
    canvas.drawFastHLine(sx - 3, y - 1, 7, C_WHITE);
    canvas.drawFastVLine(sx, y - 4, 8, C_WHITE);
  }
}

void checkItems() {
  // Check only nearby objects; collecting while airborne requires proximity
  int ci = (int)lroundf((carX - 115.0f) / 88.0f);
  for (int i = ci - 1; i <= ci + 1; ++i) {
    if (i < 0 || i >= COIN_MAX || coinTaken[i]) continue;
    float x = coinPosition(i);
    if (fabsf(carX - x) < 17.0f &&
        fabsf((carY - 10.0f) - (groundY(x) - 28.0f)) < 25.0f) {
      coinTaken[i] = true;
      ++score;
      chirp(1250 + (score % 4) * 130, 65);
    }
  }

  int fi = (int)lroundf((carX - 305.0f) / 475.0f);
  for (int i = fi - 1; i <= fi + 1; ++i) {
    if (i < 0 || i >= CAN_MAX || canTaken[i]) continue;
    float x = canPosition(i);
    if (fabsf(carX - x) < 19.0f &&
        fabsf((carY - 10.0f) - (groundY(x) - 29.0f)) < 24.0f) {
      canTaken[i] = true;
      fuel = clampFloat(fuel + 44.0f, 0.0f, 100.0f);
      chirp(920, 175);
    }
  }
}

// ---- Detailed, tilted vehicle ----
void drawWheel(int16_t x, int16_t y, float spin) {
  canvas.fillCircle(x, y + 1, 7, C_DARK);
  canvas.fillCircle(x, y, 6, C_WHEEL);
  canvas.drawCircle(x, y, 6, C_GRAY);
  canvas.fillCircle(x, y, 3, C_RIM);
  canvas.fillCircle(x, y, 1, C_DARK);
  for (int i = 0; i < 4; ++i) {
    float a = spin + i * PI_F * 0.5f;
    canvas.drawPixel(x + (int)lroundf(cosf(a) * 3.0f),
                     y + (int)lroundf(sinf(a) * 3.0f), C_DARK);
  }
}

void drawVehicle(int cx, int cy, float angle, float spin, float speed) {
  // Exhaust dust/smoke, only while driving on ground
  if (!inAir && speed > 32.0f) {
    uint32_t t = millis() / 75;
    for (int j = 0; j < 3; ++j) {
      int xx = cx - 24 - j * 5 - (int)((t + j * 3) % 7);
      int yy = cy + 3 - j * 2;
      canvas.fillCircle(xx, yy, j == 0 ? 2 : 1, C_ROCK);
    }
  }

  // Unsprung wheels follow the terrain; the body reacts on a damped spring.
  // Unlike the earlier version, the entire car does not bob as a rigid block.
  float bodyY = cy + suspension - 4.0f;
  int16_t rx, ry, fx, fy, rearMountX, rearMountY, frontMountX, frontMountY;
  rotateXY(cx, cy - 4.0f, angle, -13, 4, rx, ry);
  rotateXY(cx, cy - 4.0f, angle, 13, 4, fx, fy);
  rotateXY(cx, bodyY, angle, -11, 1, rearMountX, rearMountY);
  rotateXY(cx, bodyY, angle, 11, 1, frontMountX, frontMountY);
  canvas.drawLine(rearMountX, rearMountY, rx, ry, C_GRAY);
  canvas.drawLine(frontMountX, frontMountY, fx, fy, C_GRAY);
  canvas.fillCircle(rearMountX, rearMountY, 2, C_SILVER);
  canvas.fillCircle(frontMountX, frontMountY, 2, C_SILVER);

  // Chassis shadow behind the bodywork
  rotatedQuad(cx, bodyY, angle, -19, -2, 20, 6, C_DARK);

  // Main body, hood and dark bumper accents
  rotatedQuad(cx, bodyY, angle, -20, -6, 20, 1, C_RED_DARK);
  rotatedQuad(cx, bodyY, angle, -19, -8, 18, -2, C_RED);
  rotatedQuad(cx, bodyY, angle, -19, -7, -10, -5, C_HIGHLIGHT);
  rotatedQuad(cx, bodyY, angle, 10, -7, 19, -5, C_HIGHLIGHT);

  // Cabin, roof, front and rear glass
  rotatedQuad(cx, bodyY, angle, -10, -16, 7, -8, C_RED_DARK);
  rotatedQuad(cx, bodyY, angle, -8, -15, 6, -10, C_GLASS);
  rotatedQuad(cx, bodyY, angle, -1, -15, 0, -10, C_SILVER); // window divider
  rotatedLine(cx, bodyY, angle, -10, -16, 7, -16, C_HIGHLIGHT);
  rotatedLine(cx, bodyY, angle, 7, -16, 11, -8, C_RED);
  rotatedLine(cx, bodyY, angle, -10, -16, -13, -8, C_RED);

  // Headlight / rear light
  int16_t px, py;
  rotateXY(cx, bodyY, angle, 19, -5, px, py);
  canvas.fillCircle(px, py, 2, C_SUN);
  rotateXY(cx, bodyY, angle, -19, -5, px, py);
  canvas.fillRect(px - 1, py - 1, 2, 3, C_RED_DARK);

  // Wheel hubs stay positioned independently of spring-compressed body.
  drawWheel(rx, ry, spin);
  drawWheel(fx, fy, spin);
}

// ---- On-screen game HUD ----
void drawHUD(bool gas, bool brake) {
  canvas.fillRect(0, 0, W, 19, C_DARK);
  canvas.drawFastHLine(0, 18, W, C_SILVER);
  canvas.setTextSize(1);
  canvas.setTextWrap(false);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(3, 5);
  canvas.print("F");
  canvas.drawRoundRect(14, 5, 46, 9, 2, C_GRAY);
  int fill = (int)lroundf(clampFloat(fuel, 0.0f, 100.0f) * 0.42f);
  if (fill > 0) canvas.fillRect(16, 7, fill, 5, fuel < 25 ? C_RED : C_GREEN);
  canvas.fillCircle(72, 9, 5, C_GOLD);
  canvas.drawCircle(72, 9, 5, C_HIGHLIGHT);
  canvas.setCursor(79, 5);
  canvas.print(score);
  canvas.setCursor(116, 5);
  canvas.print((int)(distanceBest / 10.0f));
  canvas.print("m");

  canvas.fillRoundRect(109, 109, 51, 19, 4, C_DARK);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(114, 115);
  canvas.print((int)(fabsf(carSpeed) * 0.45f));
  canvas.print("kmh");

  if (totalFlips > 0 && (int32_t)(millis() - flipTextUntil) < 0) {
    canvas.fillRoundRect(55, 22, 52, 13, 3, C_DARK);
    canvas.setTextColor(C_GOLD);
    canvas.setCursor(61, 25);
    canvas.print("FLIP!");
  }

  // Pedal status light along bottom left
  canvas.fillRoundRect(2, 114, 38, 13, 3, brake ? C_RED : C_DARK);
  canvas.fillRoundRect(43, 114, 34, 13, 3, gas ? C_GREEN : C_DARK);
  canvas.setTextColor(C_WHITE);
  canvas.setCursor(7, 117); canvas.print("STOP");
  canvas.setCursor(50, 117); canvas.print("GO");
}

void drawCentered(const char *s, int y, int size, uint16_t color) {
  canvas.setTextWrap(false);
  canvas.setTextSize(size);
  canvas.setTextColor(color);
  int textWidth = (int)strlen(s) * 6 * size;
  canvas.setCursor((W - textWidth) / 2, y);
  canvas.print(s);
}

void drawPlayScene(bool gas, bool brake, uint32_t now) {
  drawBackdrop(cameraX);
  drawTrack(cameraX);
  drawItems(cameraX, now);
  int sx = (int)lroundf(carX - cameraX);
  drawVehicle(sx, (int)lroundf(carY), carTilt, tireSpin, fabsf(carSpeed));
  drawHUD(gas, brake);
}

void drawTitleScene() {
  drawBackdrop(0.0f);
  drawTrack(0.0f);
  canvas.fillRoundRect(9, 22, 142, 88, 8, C_DARK);
  canvas.drawRoundRect(9, 22, 142, 88, 8, C_GOLD);
  drawCentered("HILL CLIMB", 29, 2, C_GOLD);
  drawCentered("RACING MINI 4.0", 49, 1, C_WHITE);
  drawVehicle(80, 77, -0.10f, 0.0f, 0.0f);
  drawCentered("AIR: GAS UP BRAKE DOWN", 94, 1, C_CLOUD);
  drawCentered("PRESS GAS TO START", 114, 1, C_WHITE);
}

void drawGameOverScene() {
  drawPlayScene(false, false, millis());
  canvas.fillRoundRect(13, 27, 134, 77, 8, C_DARK);
  canvas.drawRoundRect(13, 27, 134, 77, 8, C_RED);
  drawCentered("GAME OVER", 34, 2, C_GOLD);
  drawCentered(endReason, 56, 1, C_WHITE);
  char text[32];
  snprintf(text, sizeof(text), "%dm  /  %d COINS", (int)(distanceBest / 10.0f), score);
  drawCentered(text, 72, 1, C_WHITE);
  drawCentered("RELEASE THEN PRESS GAS", 89, 1, C_GREEN);
  if (totalFlips) {
    canvas.setCursor(59, 101);
    canvas.setTextSize(1);
    canvas.setTextColor(C_GOLD);
    canvas.print("FLIPS: "); canvas.print(totalFlips);
  }
}

// Exactly one TFT update per rendered frame. Physical display is NOT cleared.
void showCanvas() {
  tft.drawRGBBitmap(0, 0, canvas.getBuffer(), W, H);
}

void resetRun() {
  memset(coinTaken, 0, sizeof(coinTaken));
  memset(canTaken, 0, sizeof(canTaken));
  carX = 36.0f;
  carSpeed = 0.0f;
  carY = axleGround(carX);
  fallSpeed = 0.0f;
  carTilt = roadTilt(carX);
  tireSpin = 0.0f;
  tireSpinSpeed = 0.0f;
  suspension = 0.0f;
  suspensionSpeed = 0.0f;
  pitchSpeed = 0.0f;
  flightTime = 0.0f;
  takeoffAngle = 0.0f;
  flipsThisJump = 0;
  totalFlips = 0;
  flipTextUntil = 0;
  cameraX = 0.0f;
  fuel = 100.0f;
  distanceBest = 0.0f;
  inAir = false;
  score = 0;
  mode = MODE_DRIVE;
  lastFrame = millis();
  lastPhysics = lastFrame;
  chirp(1050, 110);
}

void finishRun(const char *why) {
  if (mode != MODE_DRIVE) return;
  endReason = why;
  mode = MODE_OVER;
  chirp(200, 350);
  drawGameOverScene();
  showCanvas();
  Serial.print("[GAME OVER] "); Serial.println(why);
}

// ---- Driving physics 4.0: real 360-degree flips and roof crashes ----
// X grows right, Y grows DOWN. Negative pitch rotates the nose upward.
// Ground alignment, inertia, ramp launch, airborne rotation and collision
// detection are independent of the 25 FPS full-frame rendering.
void stepPhysics(float dt, bool gas, bool brake, uint32_t now) {
  constexpr float GRAVITY = 465.0f;
  constexpr float TOP_SPEED = 183.0f;
  constexpr float REVERSE_LIMIT = -64.0f;
  constexpr float TIRE_RADIUS = 6.0f;
  constexpr float FULL_TURN_RAD = 2.0f * PI_F;

  const float previousX = carX;
  const float oldRoadY = axleGround(previousX);
  const float slope = (groundY(carX + 7.0f) - groundY(carX - 7.0f)) / 14.0f;
  const float incline = atanf(slope);
  const float speedAbs = fabsf(carSpeed);
  const bool driving = gas && !brake && fuel > 0.0f;

  float acceleration = 0.0f;
  if (!inAir) {
    acceleration = 250.0f * sinf(incline);
    if (speedAbs > 0.4f) {
      acceleration -= (11.0f + 0.0020f * speedAbs * speedAbs)
                       * (carSpeed > 0 ? 1.0f : -1.0f);
    } else {
      acceleration -= carSpeed * 12.0f;
    }
    const float traction = clampFloat(1.0f - fabsf(slope) * 0.23f, 0.66f, 1.0f);
    if (driving) {
      const float powerBand = clampFloat(1.0f - fmaxf(0.0f, carSpeed) / 270.0f,
                                         0.33f, 1.0f);
      acceleration += 220.0f * traction * powerBand;
    }
    if (brake) {
      if (carSpeed > 4.0f) acceleration -= 330.0f * traction;
      else acceleration -= 112.0f * traction;
    }
  } else {
    // Pedals do not create forward thrust while airborne.
    acceleration = -carSpeed * 0.06f;
  }

  carSpeed = clampFloat(carSpeed + acceleration * dt, REVERSE_LIMIT, TOP_SPEED);
  if (!gas && !brake && fabsf(carSpeed) < 0.25f && fabsf(slope) < 0.03f) {
    carSpeed = 0.0f;
  }
  carX += carSpeed * dt;
  if (carX < 22.0f) { carX = 22.0f; carSpeed = 0.0f; }
  if (carX > 20700.0f) { carX = 20700.0f; carSpeed = 0.0f; }

  if (!inAir) {
    tireSpinSpeed += ((carSpeed / TIRE_RADIUS) - tireSpinSpeed)
                    * clampFloat(dt * 19.0f, 0.0f, 1.0f);
  } else {
    tireSpinSpeed += ((gas ? 5.0f : 0.0f) - (brake ? 11.0f : 0.0f)
                      - tireSpinSpeed * 0.65f) * dt;
  }
  tireSpin += tireSpinSpeed * dt;
  distanceBest = fmaxf(distanceBest, carX - 36.0f);

  if (driving) {
    fuel -= (0.52f + speedAbs * 0.0080f) * dt;
    if (buzzerUntil == 0 && now - lastMotorSound > 270) {
      chirp((uint32_t)(120.0f + clampFloat(speedAbs * 2.0f, 0.0f, 370.0f)), 12);
      lastMotorSound = now;
    }
  } else {
    fuel -= 0.10f * dt;
  }
  fuel = clampFloat(fuel, 0.0f, 100.0f);

  const float newRoadY = axleGround(carX);
  const float groundVerticalSpeed = (newRoadY - oldRoadY) / dt;
  const float roadPitch = roadTilt(carX);
  const bool alreadyAirborne = inAir;

  if (!inAir) {
    // Launch when the road curves away faster than gravity can pull the
    // vehicle down. Fast uphill climbs provide the strongest backward flips.
    const float roadDownAccel = (groundVerticalSpeed - fallSpeed) / dt;
    const float ballisticY = carY + fallSpeed * dt + 0.5f * GRAVITY * dt * dt;
    const bool crestLaunch = fabsf(carSpeed) > 74.0f &&
                             roadDownAccel > GRAVITY * 0.91f &&
                             newRoadY > ballisticY + 0.0005f;
    if (crestLaunch) {
      inAir = true;
      flightTime = 0.0f;
      takeoffAngle = carTilt;
      flipsThisJump = 0;
      carY = ballisticY;
      fallSpeed = fminf(fallSpeed + GRAVITY * dt - 23.0f,
                        -76.0f - 30.0f * clampFloat((fabsf(carSpeed) - 80.0f) / 100.0f, 0.0f, 1.0f)); // ramp lift / spring release
      suspensionSpeed -= 8.0f;

      // Momentum and weight transfer rotate the chassis backwards at speed.
      // Gentle crests keep some control; fast ascending crests can overturn.
      const float upwardSlope = clampFloat(-incline, 0.0f, 0.9f);
      const float speedFactor = clampFloat((fabsf(carSpeed) - 72.0f) / 105.0f,
                                           0.0f, 1.0f);
      const float liftTorque = (1.2f + 5.0f * upwardSlope +
                                (driving ? 1.6f : 0.0f)) * speedFactor;
      pitchSpeed = clampFloat(pitchSpeed - liftTorque, -11.0f, 11.0f);
      chirp(470, 45);
    } else {
      carY = newRoadY;
      fallSpeed = groundVerticalSpeed;
    }
  }

  if (inAir) {
    if (alreadyAirborne) {
      flightTime += dt;
      fallSpeed = clampFloat(fallSpeed + GRAVITY * dt, -300.0f, 420.0f);
      carY += fallSpeed * dt;
    }

    // FULL ROTATION ENABLED: never clamp carTilt to +/- 180 degrees.
    // Holding GAS tilts backward; holding BRAKE tilts forward.
    // Existing angular momentum persists when buttons are released.
    pitchSpeed += ((brake ? 12.5f : 0.0f) - (gas ? 10.0f : 0.0f)
                   - pitchSpeed * 0.22f) * dt;
    pitchSpeed = clampFloat(pitchSpeed, -12.0f, 12.0f);
    carTilt += pitchSpeed * dt;

    // Award a completed full 360-degree rotation (forward or backward).
    const int fullTurns = (int)floorf(fabsf(carTilt - takeoffAngle) / FULL_TURN_RAD);
    if (fullTurns > flipsThisJump) {
      totalFlips += fullTurns - flipsThisJump;
      flipsThisJump = fullTurns;
      flipTextUntil = now + 1300;
      chirp(1560, 105);
      Serial.printf("[FLIP] Completed %d full rotation(s), total=%d\n",
                    fullTurns, totalFlips);
    }

    // Let the wheels clear the ramp on the first two/three physics ticks.
    // The roof and both tyres are then checked against the REAL hill height.
    if (flightTime > 0.025f) {
      const float relativeAngle = wrapAngle(carTilt - roadPitch);
      const float angleAbs = fabsf(relativeAngle);

      // Roof top corners are body-relative: chassis center is (carX,carY-4).
      // On inversion the roof extends downward and strikes the hill FIRST.
      const float roofL = contactDepth(-10.0f, -20.0f);
      const float roofR = contactDepth(  7.0f, -20.0f);
      const float bumperL = contactDepth(-19.0f, -8.0f);
      const float bumperR = contactDepth( 19.0f, -8.0f);
      const bool roofHit = roofL >= -0.5f || roofR >= -0.5f;
      const bool bumperHit = bumperL >= -0.5f || bumperR >= -0.5f;

      // True wheel contact: wheels are 6px radius, with axles at y=carY-4
      // plus the rotated local offset (+4) relative to chassis center.
      const float rearDepth = contactDepth(-13.0f, 0.0f) + TIRE_RADIUS;
      const float frontDepth = contactDepth(13.0f, 0.0f) + TIRE_RADIUS;
      const float wheelPenetration = fmaxf(rearDepth, frontDepth);
      const bool wheelHit = wheelPenetration >= 0.0f;

      // The car must land on its TYRES. Sideways / roof-first impacts crash.
      // A complete 360 returns relativeAngle to zero and can land safely.
      if ((roofHit && angleAbs > 1.25f) ||
          (bumperHit && angleAbs > 2.00f) ||
          (wheelHit && angleAbs > 1.65f)) {
        finishRun("ROOF CRASH!");
        return;
      }

      if (wheelHit && flightTime > 0.09f &&
          fallSpeed > groundVerticalSpeed + 5.0f) {
        const float impact = fmaxf(0.0f, fallSpeed - groundVerticalSpeed);
        if ((impact > 305.0f && fabsf(carSpeed) > 45.0f) ||
            (angleAbs > 1.10f && impact > 130.0f)) {
          finishRun("HARD LANDING");
          return;
        }
        // Resolve downward penetration without teleporting to a road line.
        carY -= clampFloat(wheelPenetration, 0.0f, 10.0f);
        inAir = false;
        flightTime = 0.0f;
        fallSpeed = groundVerticalSpeed;
        suspensionSpeed += clampFloat(impact * 0.13f, 0.0f, 24.0f);
        carSpeed *= clampFloat(1.0f - impact * 0.0004f, 0.84f, 1.0f);
        pitchSpeed *= 0.45f;
        if (impact > 30.0f) chirp(330, 52);
      }
    }
  } else {
    // Grounded chassis follows the slope on springs; acceleration provides
    // a subtle wheelie effect on a steep uphill without artificial flipping.
    const float ascent = clampFloat(-roadPitch, 0.0f, 0.7f);
    const float wheelie = driving ?
        ascent * clampFloat((fabsf(carSpeed) - 55.0f) / 210.0f, 0.0f, 0.30f) : 0.0f;
    const float desiredPitch = roadPitch - wheelie;
    const float error = wrapAngle(desiredPitch - carTilt);
    pitchSpeed += (105.0f * error - 17.0f * pitchSpeed) * dt;
    pitchSpeed = clampFloat(pitchSpeed, -5.0f, 5.0f);
    carTilt += pitchSpeed * dt;
  }

  const float vibration = inAir ? 0.0f :
      0.25f * sinf(tireSpin * 1.8f) *
      clampFloat(fabsf(carSpeed) / 150.0f, 0.0f, 1.0f);
  suspensionSpeed += (-105.0f * (suspension - vibration)
                      - 18.0f * suspensionSpeed) * dt;
  suspension += suspensionSpeed * dt;
  suspension = clampFloat(suspension, -2.5f, 2.8f);

  checkItems();
  if (fuel <= 0.0f) {
    finishRun("OUT OF FUEL");
    return;
  }
  if (fuel < 18.0f && now - lastLowFuelSound > 1700 && buzzerUntil == 0) {
    chirp(680, 100);
    lastLowFuelSound = now;
  }

  const float targetCam = fmaxf(0.0f, carX - 49.0f);
  cameraX += (targetCam - cameraX) * clampFloat(dt * 7.0f, 0.0f, 1.0f);
}

void setup() {
  Serial.begin(115200);
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  pinMode(GAS_PIN, INPUT_PULLUP);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BUZZER_PIN, 1000, 8);
#else
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZER_PIN, 0);
#endif
  buzzerHz(0);

  // Hardware SPI pins exactly as connected by the user.
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.setSPISpeed(TFT_SPI_HZ);
  tft.setTextWrap(false);

  if (tft.width() != W || tft.height() != H) {
    Serial.println("ERROR: expected landscape ST7735 160x128 display");
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 0);
    tft.print("Check TFT type!");
    while (true) delay(1000);
  }
  if (canvas.getBuffer() == nullptr) {
    Serial.println("ERROR: GFXcanvas16 memory allocation failed");
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(0, 0);
    tft.print("NO FRAME RAM");
    while (true) delay(1000);
  }

  C_SKY1      = tft.color565(76, 168, 232);
  C_SKY2      = tft.color565(117, 200, 239);
  C_SKY3      = tft.color565(171, 219, 238);
  C_SUN       = tft.color565(255, 226, 137);
  C_CLOUD     = tft.color565(241, 250, 255);
  C_MOUNT1    = tft.color565(148, 186, 209);
  C_MOUNT2    = tft.color565(105, 159, 168);
  C_DIRT      = tft.color565(159, 109, 62);
  C_DIRT2     = tft.color565(125, 79, 50);
  C_SOIL      = tft.color565(96, 62, 46);
  C_GRASS     = tft.color565(107, 207, 73);
  C_GRASS2    = tft.color565(46, 154, 70);
  C_ROCK      = tft.color565(156, 139, 118);
  C_WHITE     = tft.color565(245, 247, 253);
  C_DARK      = tft.color565(22, 33, 49);
  C_GRAY      = tft.color565(72, 87, 101);
  C_RED       = tft.color565(247, 79, 42);
  C_RED_DARK  = tft.color565(154, 41, 36);
  C_GLASS     = tft.color565(148, 218, 234);
  C_GLASS2    = tft.color565(89, 151, 174);
  C_GOLD      = tft.color565(250, 187, 42);
  C_HIGHLIGHT = tft.color565(255, 231, 134);
  C_GREEN     = tft.color565(57, 212, 101);
  C_FUEL      = tft.color565(226, 61, 41);
  C_WHEEL     = tft.color565(36, 44, 55);
  C_RIM       = tft.color565(184, 202, 208);
  C_SILVER    = tft.color565(110, 128, 146);

  drawTitleScene();
  showCanvas();
  lastFrame = millis();
  lastPhysics = lastFrame;
  Serial.println("HILL CLIMB MINI 4.0.1 READY | 100 Hz physics + double-buffered TFT");
  Serial.println("TFT SCK=12 MOSI=11 DC=9 RST=10 CS=8");
  Serial.println("BUTTONS: BRAKE GPIO4, GAS GPIO5 | PIEZO GPIO6");
  Serial.println("Press GAS to start, release then GAS to restart.");
}

void loop() {
  uint32_t now = millis();
  serviceSound(now);

  const bool gas = (digitalRead(GAS_PIN) == LOW);
  const bool brake = (digitalRead(BRAKE_PIN) == LOW);
  const bool gasPressed = gas && !gasBefore;
  gasBefore = gas;

  if (mode != MODE_DRIVE) {
    if (gasPressed) { resetRun(); now = millis(); }
    else { delay(2); return; }
  }

  // Run simulation with a stable timestep, regardless of the time taken
  // for the 160x128 TFT transfer. Cap catch-up to avoid the spiral of death.
  int physicsSteps = 0;
  while ((uint32_t)(now - lastPhysics) >= PHYSICS_MS && physicsSteps < 8) {
    lastPhysics += PHYSICS_MS;
    stepPhysics(PHYSICS_MS / 1000.0f, gas, brake, now);
    ++physicsSteps;
    if (mode != MODE_DRIVE) return;
  }
  if ((uint32_t)(now - lastPhysics) > 150) lastPhysics = now;

  if ((uint32_t)(now - lastFrame) >= FRAME_MS) {
    lastFrame = now;
    drawPlayScene(gas, brake, now);
    showCanvas(); // One complete image transfer. No per-frame TFT clearing.
  } else {
    delay(1);
  }

  if (now - lastSerialStatus > 2500) {
    Serial.printf("X: %.0f | speed: %.0f | fuel: %.0f%% | coins: %d | air: %d | pitch: %.2f | flips: %d\n",
                  carX, carSpeed, fuel, score, inAir ? 1 : 0, carTilt, totalFlips);
    lastSerialStatus = now;
  }
}
