# Warrior Libraries

Custom Arduino libraries for the Warrior robot. Each library lives in its own
sub-folder and follows the standard Arduino library layout:

```
LibraryName/
├── library.properties   # name, version, dependencies
└── src/
    ├── LibraryName.h
    └── LibraryName.cpp
```

---

## Getting the Libraries to Work in Arduino IDE

### Step 1 — Set the Sketchbook Location

The Arduino IDE automatically discovers libraries that live inside your
**Sketchbook** folder.  This repo is already laid out to match that structure:

```
Warrior/          ← set this as your Sketchbook location
├── libraries/    ← auto-discovered by the IDE
└── sketches/     ← open .ino files from here
```

1. Open Arduino IDE.
2. Go to **File → Preferences** (Windows/Linux) or **Arduino IDE → Settings** (macOS).
3. In the **Sketchbook location** field, enter the full path to the `Warrior/`
   folder, e.g.:
   ```
   C:\LS\Microprocessor_code\Warrior
   ```
4. Click **OK** and **restart the IDE**.

After restarting, every library in `Warrior/libraries/` is available to any
sketch you open.

> **Note:** Changing the Sketchbook location moves where the IDE looks for
> *everything* (libraries, boards, etc.).  If you have other projects that
> rely on the default location, consider copying the library folders to
> `Documents/Arduino/libraries/` instead (see the alternative below).

---

### Step 2 — Install External Dependencies

Some libraries in this project depend on third-party packages that must be
installed via the IDE's **Library Manager**:

| Library | Required by | How to install |
|---|---|---|
| **Adafruit NeoPixel** | FanControl, LEDControl | Library Manager → search `Adafruit NeoPixel` → Install |
| **ESP32Servo** | MotorControl | Library Manager → search `ESP32Servo` → Install |

To open Library Manager: **Sketch → Include Library → Manage Libraries…**
(or `Ctrl+Shift+I`).

---

### Step 3 — Include a Library in a Sketch

Once the Sketchbook location is set correctly, include any library with a
standard `#include`:

```cpp
#include <FanControl.h>
#include <MotorControl.h>
#include <RadioLink.h>
```

You can also use **Sketch → Include Library** and click the library name; the
IDE will insert the `#include` line for you.

---

## Alternative: Copy Libraries to the Default Location

If you do not want to change your Sketchbook location, copy (or symlink) each
library folder into `Documents/Arduino/libraries/`:

```
Documents/
└── Arduino/
    └── libraries/
        ├── FanControl/
        ├── MotorControl/
        └── ...
```

---

## Available Libraries

| Library | Description |
|---|---|
| **FanControl** | NeoPixel LED ring colours, moving-block animation, and PWM fan speed control |
| **LEDControl** | General-purpose LED control |
| **MotorControl** | Drive and steering motor output (Flipsky, SparkMax) |
| **PID** | Generic PID control loop |
| **RadioLink** | PPM input from RC receiver |
| **SerialProtocol** | Framed ASCII serial messaging `<MSGTYPE,d1,d2,...>` |
