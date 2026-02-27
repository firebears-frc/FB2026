# FRC Robot Controller Mapping

## Reference Documentation

| Resource | Link |
|---|---|
| 🗺️ Xbox Series X\|S Controller Diagram | [support.xbox.com — Get to know your controller](https://support.xbox.com/en-US/help/hardware-network/controller/get-to-know-your-xbox-series-x-s-controller) |
| 🗺️ Xbox One Controller Diagram | [support.xbox.com — Xbox One Wireless Controller](https://support.xbox.com/en-US/help/hardware-network/controller/xbox-one-wireless-controller) |
| 📄 WPILib XboxController API (Java) | [github.wpilib.org — XboxController](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/XboxController.html) |
| 📄 WPILib Joystick & Controller Docs | [docs.wpilib.org — Joysticks](https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html) |
| 🎮 Xbox Controller Button Names (community reference) | [github.com/palmerj — Button Names & Layout](https://gist.github.com/palmerj/586375bcc5bc83ccdaf00c6f5f863e86) |

---

## Xbox Controller — Driver (Port 0)

> Controls robot drivetrain and primary movement.

| Button / Axis | Input | Action |
|---|---|---|
| Left Stick (Y-Axis) | Axis 1 | Drive forward / backward |
| Right Stick (X-Axis) | Axis 4 | Rotate left / right |
| Left Trigger | Axis 2 | Slow mode (reduced speed) |
| Right Trigger | Axis 3 | Boost / turbo speed |
| Left Bumper (LB) | Button 5 | Shift to low gear |
| Right Bumper (RB) | Button 6 | Shift to high gear |
| A Button | Button 1 | Auto-align to target |
| B Button | Button 2 | Cancel auto routine |
| X Button | Button 3 | — |
| Y Button | Button 4 | — |
| Start | Button 8 | Reset gyro / field-relative heading |
| Back | Button 7 | — |
| D-Pad Up | POV 0° | — |
| D-Pad Down | POV 180° | — |

---

## Xbox Controller — Operator (Port 1)

> Controls manipulator subsystems (arm, intake, shooter, etc.).

| Button / Axis | Input | Action |
|---|---|---|
| Left Stick (Y-Axis) | Axis 1 | Arm up / down (manual) |
| Right Stick (Y-Axis) | Axis 5 | Elevator up / down (manual) |
| Left Trigger | Axis 2 | Intake in |
| Right Trigger | Axis 3 | Intake out / eject |
| Left Bumper (LB) | Button 5 | Shooter rev up |
| Right Bumper (RB) | Button 6 | Fire / shoot |
| A Button | Button 1 | Go to floor pickup position |
| B Button | Button 2 | Go to stow position |
| X Button | Button 3 | Go to low goal position |
| Y Button | Button 4 | Go to high goal position |
| Start | Button 8 | Reset arm encoder |
| Back | Button 7 | Emergency stop subsystems |
| D-Pad Up | POV 0° | Fine-tune arm up |
| D-Pad Down | POV 180° | Fine-tune arm down |
| D-Pad Left | POV 270° | Fine-tune wrist left |
| D-Pad Right | POV 90° | Fine-tune wrist right |

---

## Notes

- **Button numbering** follows the WPILib `XboxController` class convention (1-indexed).
- **Axis numbering** follows WPILib defaults (0-indexed).
- Triggers return values from `0.0` (released) to `1.0` (fully pressed). A threshold of `0.1` is recommended to avoid drift.
- All preset positions (A/B/X/Y on operator) use PID-controlled motion profiling.
- Slow mode scales output by **40%**; boost mode scales output by **100%**.

---

## WPILib Button Reference (XboxController)

```java
XboxController controller = new XboxController(0);

// Buttons
new JoystickButton(controller, XboxController.Button.kA.value)
new JoystickButton(controller, XboxController.Button.kB.value)
new JoystickButton(controller, XboxController.Button.kX.value)
new JoystickButton(controller, XboxController.Button.kY.value)
new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
new JoystickButton(controller, XboxController.Button.kRightBumper.value)
new JoystickButton(controller, XboxController.Button.kBack.value)
new JoystickButton(controller, XboxController.Button.kStart.value)

// Triggers (as buttons with threshold)
new Trigger(() -> controller.getLeftTriggerAxis() > 0.1)
new Trigger(() -> controller.getRightTriggerAxis() > 0.1)

// D-Pad (POV)
new POVButton(controller, 0)    // Up
new POVButton(controller, 90)   // Right
new POVButton(controller, 180)  // Down
new POVButton(controller, 270)  // Left
```
