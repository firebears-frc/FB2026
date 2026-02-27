# FRC Robot Controller Mapping
## Thrustmaster T.16000M FCS Space Sim Duo

> The Space Sim Duo consists of **two individual T.16000M FCS joysticks** — one for each hand.  
> In FRC, each stick connects as a **separate USB device** and appears as its own port in the Driver Station.  
> ⚠️ Always verify port assignments in the **FRC Driver Station USB Devices tab** before a match.

## Reference Documentation

| Resource | Link |
|---|---|
| 📄 Official User Manual (PDF) | [T.16000M User Manual](https://ts.thrustmaster.com/download/accessories/Manuals/T16000M/T16000M-User_manual.pdf) |
| 🌐 Official Support Page | [support.thrustmaster.com — T.16000M FCS](https://support.thrustmaster.com/en/product/t16000mfcs-en/) |
| 🗺️ Mapping Example — Duo (Elite Dangerous) | [View PNG](https://ts.thrustmaster.com/download/pictures/pcmac/T16000M/EliteD_T16000MDUO_Mapping.png) |
| 🗺️ Mapping Example — Duo (MechWarrior) | [View JPG](https://ts.thrustmaster.com/download/pictures/pcmac/T16000M/Mapping-T16000MDuo_MechWarrior.jpg) |
| 🗺️ Mapping Example — Duo (Everspace) | [View JPG](https://ts.thrustmaster.com/download/pictures/pcmac/T16000M/Mapping-T16000MDuo_EverSpace.jpg) |

> 💡 The **User Manual PDF** contains the most useful labeled diagram showing all button numbers and axis locations on both the stick body and base, for both right-handed and left-handed configurations.

---

## Physical Layout — T.16000M FCS Stick (per stick)

| # | Control | Description |
|---|---|---|
| 1 | Digital Trigger | Index finger trigger |
| 2 | POV Hat Switch | 8-direction thumb hat (top of stick) |
| 3 | Throttle Slider | Side slider (Axis: Slider0) |
| 4 | Twist / Rudder | Rotate stick left/right (Axis: RZ) |
| 5–16 | Action Buttons | 4 on stick body + 12 on base |

> **Note on base buttons:** The 12 base buttons change physical position depending on the **Right-Handed / Left-Handed selector switch** on the underside of the base, but their button IDs remain the same.

---

## Axes — Per Stick

| Axis Name | WPILib Axis ID | Range | Description |
|---|---|---|---|
| X Axis | Axis 0 | 0 → 16383 | Left/Right stick deflection |
| Y Axis | Axis 1 | 0 → 16383 | Forward/Back stick deflection |
| RZ (Twist) | Axis 2 | 0 → 16383 | Stick rotation (rudder) |
| Slider0 | Axis 3 | 0 → 255 | Throttle slider |

> Y axis is typically **inverted** — pushing forward gives a negative value. Negate in code: `-joystick.getRawAxis(1)`

---

## Buttons — Per Stick

| Button | WPILib ID | Location | Action |
|---|---|---|---|
| Trigger | Button 1 | Index finger | _TODO_ |
| Button 2 | Button 2 | Thumb (stick top) | _TODO_ |
| Button 3 | Button 3 | Stick body | _TODO_ |
| Button 4 | Button 4 | Stick body | _TODO_ |
| Button 5 | Button 5 | Base | _TODO_ |
| Button 6 | Button 6 | Base | _TODO_ |
| Button 7 | Button 7 | Base | _TODO_ |
| Button 8 | Button 8 | Base | _TODO_ |
| Button 9 | Button 9 | Base | _TODO_ |
| Button 10 | Button 10 | Base | _TODO_ |
| Button 11 | Button 11 | Base | _TODO_ |
| Button 12 | Button 12 | Base | _TODO_ |
| Button 13 | Button 13 | Base | _TODO_ |
| Button 14 | Button 14 | Base | _TODO_ |
| Button 15 | Button 15 | Base | _TODO_ |
| Button 16 | Button 16 | Base | _TODO_ |

---

## POV Hat Switch — Per Stick

| Direction | Angle | Action |
|---|---|---|
| Up | 0° | _TODO_ |
| Up-Right | 45° | _TODO_ |
| Right | 90° | _TODO_ |
| Down-Right | 135° | _TODO_ |
| Down | 180° | _TODO_ |
| Down-Left | 225° | _TODO_ |
| Left | 270° | _TODO_ |
| Up-Left | 315° | _TODO_ |
| Unpressed | -1 | — |

---

## Driver Assignment (Firebears — Team 2846)

### Right Stick — Driver (Port 0)

| Input | Action |
|---|---|
| X Axis (Axis 0) | _TODO_ |
| Y Axis (Axis 1) | _TODO_ |
| Twist/RZ (Axis 2) | _TODO_ |
| Throttle (Axis 3) | _TODO_ |
| Trigger (Button 1) | _TODO_ |
| Button 2 | _TODO_ |
| Button 3 | _TODO_ |
| Button 4 | _TODO_ |
| Buttons 5–16 (Base) | _TODO_ |
| POV Hat | _TODO_ |

### Left Stick — Operator (Port 1)

| Input | Action |
|---|---|
| X Axis (Axis 0) | _TODO_ |
| Y Axis (Axis 1) | _TODO_ |
| Twist/RZ (Axis 2) | _TODO_ |
| Throttle (Axis 3) | _TODO_ |
| Trigger (Button 1) | _TODO_ |
| Button 2 | _TODO_ |
| Button 3 | _TODO_ |
| Button 4 | _TODO_ |
| Buttons 5–16 (Base) | _TODO_ |
| POV Hat | _TODO_ |

---

## Notes

- **Button IDs are 1-indexed** as used by WPILib's `getRawButton()`.
- **Axis IDs are 0-indexed** as used by WPILib's `getRawAxis()`.
- The T.16000M uses **H.E.A.R.T magnetic sensors** — axes are very precise with 16-bit resolution. A small deadzone (~0.05) is still recommended for center drift.
- The **throttle slider** (Axis 3) runs from `0` (full back) to `1.0` (full forward) in WPILib's normalized scale.
- The **twist/rudder axis** (Axis 2) rotates the entire stick handle and is a full analog axis.
- Base buttons **physically swap positions** when the Left/Right-Handed switch is flipped on the bottom of the stick, but the **button IDs do not change** — always verify in Driver Station.
- ⚠️ **Confirm USB port order** by plugging in one stick at a time and checking which port it appears on in the Driver Station USB Devices tab.

---

## WPILib Code Reference

```java
// Instantiate both sticks
Joystick rightStick = new Joystick(0); // Right hand / Driver   — Port 0
Joystick leftStick  = new Joystick(1); // Left hand  / Operator — Port 1

// Read axes
double stickX    = rightStick.getRawAxis(0);           // X axis
double stickY    = -rightStick.getRawAxis(1);          // Y axis (negated: forward = positive)
double stickTwist = rightStick.getRawAxis(2);          // Twist / rudder
double throttle   = rightStick.getRawAxis(3);          // Throttle slider

// Apply deadzone helper (example)
double applyDeadzone(double value, double threshold) {
    return (Math.abs(value) > threshold) ? value : 0.0;
}
double drive = applyDeadzone(-rightStick.getRawAxis(1), 0.05);

// Read buttons
boolean triggerPressed = rightStick.getRawButton(1);   // Trigger
boolean button2        = rightStick.getRawButton(2);

// Bind a command (Command-Based)
new JoystickButton(rightStick, 1).onTrue(new ExampleCommand());

// Read the POV hat
int hat = rightStick.getPOV(); // returns angle in degrees, or -1 if unpressed
if (hat == 0)   { /* Hat Up    */ }
if (hat == 180) { /* Hat Down  */ }
if (hat == 270) { /* Hat Left  */ }
if (hat == 90)  { /* Hat Right */ }
```
