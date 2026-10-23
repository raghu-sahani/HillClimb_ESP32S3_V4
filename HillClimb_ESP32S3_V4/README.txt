HILL CLIMB MINI 4.0 - REAL FLIPS + ROOF CRASHES (ESP32-S3 + ST7735)
==================================================================
CHANGE IN V4
- Car can now rotate completely (360 degrees and beyond) during a jump.
- Faster uphill launches produce backward rotation from momentum.
- GAS in midair rotates the nose upward/backward; BRAKE rotates forward.
- Angular momentum persists with both buttons released.
- Roof and bumper collision checks against actual hills detect overturned crashes.
- Landing on wheels after a successful 360-degree flip is allowed.
- Game-over screen shows the run's flip count; airborne flip notification.
- All visual style, fuel, coins, sounds, physics timestep and TFT frame buffering preserved.

WIRING (UNCHANGED)
TFT LED = 3.3V; SCK = GPIO12; SDA/MOSI = GPIO11; DC/A0 = GPIO9;
RESET = GPIO10; CS = GPIO8; VCC = 3.3V; GND = GND.
Brake push button: GPIO4 to GND. Accelerator: GPIO5 to GND.
3.3V-compatible passive piezo buzzer: + GPIO6, - GND.

ARDUINO IDE
Board: ESP32S3 Dev Module / ESP32-S3 SuperMini
Libraries: Adafruit GFX Library, Adafruit ST7735 and ST7789 Library
Open HillClimb_ESP32S3_V4.ino. Select board and COM port; click Upload.
Serial Monitor: 115200 baud.

HOW TO PLAY
Press GAS to start; hold GAS to accelerate and climb.
Hold BRAKE to stop or reverse.
Get enough speed before a steep crest to fly and flip backwards naturally.
In the air, hold GAS to turn BACKWARDS or BRAKE to turn FORWARDS.
Do a full 360 and get the wheels back down for a safe landing.
If the roof strikes the hill, the game ends with ROOF CRASH!
Release then press GAS to restart after game over.

PERFORMANCE AND TROUBLESHOOTING
40,960-byte framebuffer with GFXcanvas16, 25 FPS target, 100 Hz physics.
Default TFT SPI = 24 MHz. If artifacts persist, try TFT_SPI_HZ = 16000000.
This version is a 2D arcade-physics simulation, not full rigid-body dynamics.
Hardware behavior should be checked after uploading to the actual board.
