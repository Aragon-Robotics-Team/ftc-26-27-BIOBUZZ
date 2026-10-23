# Keyboard driving (no Driver Hub)

Drive the robot from a laptop keyboard, for practice and testing. It is **not legal at events**: a robot must be
driven from a Driver Station there. The bridge turns itself off for good as soon as a Driver Station connects
(see [Safety](#safety)).

## Setup

1. Install the robot code as usual (a full install: Sloth's quick deploy doesn't update the page, which lives in
   `TeamCode/src/main/assets/keyboard/`).
2. Join the laptop to the robot's Wi-Fi.
3. Open **http://192.168.43.1:8080/keyboard** in Chrome, Edge, Firefox or Safari.
4. Pick an OpMode, then **Init** and **Start**, as on the Driver Station. Telemetry shows on the right.

The page has to have focus to drive. If it doesn't, it says "Click here to drive".

First time on a new laptop or after code changes: put the robot on blocks with its wheels off the ground, and check
that closing the laptop lid stops it within about a quarter second.

## Keys

These are the default keys. Each driver can change them (see [Changing keys](#changing-keys)).

| Keys | Gamepad | TeleOp |
|---|---|---|
| W A S D | 1, left stick | Drive (field-centric) |
| Q / E | 1, right stick x | Turn left / right |
| Shift | 1, LT | Hold: slow mode (driving and turning) |
| Space | 1, RT | Hold: open the gate and feed the shooter |
| I / K | 1, △ / ✕ | Toggle intake / outtake |
| U / O | 1, LB / RB | Flywheel off / on |
| J / L | 2, □ / ○ | Hive LEFT / RIGHT cell is up (hold 1 s: lock / unlock auto hive) |
| ← → | 2, dpad | Nudge turret |
| ↑ ↓ | 2, dpad | Flywheel speed |
| 7 / 8 | 1, □ / ○ | During init: blue / red alliance |
| 9 / 0 | 2, share / options | Pose reset into our LEFT / RIGHT corner |
| − / = | 1, dpad ↑ / ↓ | During init in Auto: previous / next routine |
| Esc | | Stop the OpMode (from any laptop) |

Keys you only use while setting up sit in the top row, away from the driving keys. The page labels each key for the
OpMode in use, from `tools/control-labels.json`, so the tuners are labelled too.

**Turning:** a key can't point a stick, so when TeleOp starts with the keyboard driving it turns at a rate
(`driveFieldCentric`) instead of pointing the intake (`driveTwinStick`). With a Driver Station it works as before.

**Laptop gamepad:** a USB or Bluetooth gamepad plugged into the laptop adds to gamepad 1, and a second one becomes
gamepad 2. A button counts if either the key or the gamepad has it; for sticks the bigger value wins. Use Chrome,
Edge or Safari for this: Firefox doesn't allow gamepads on a plain `http://` page.

## Changing keys

**Edit keys** opens the editor. Driving pauses while it is open (both gamepads at rest), and Esc still stops the
robot.

- Every control on both gamepads is listed, including ones with no key yet (each stick direction separately).
  Click **+** next to a control, then press the key. A control can have several keys.
- A key does one thing: binding it somewhere new takes it off the control it had, and the editor says so.
- Esc, Cmd, Ctrl, Opt and Caps Lock can't be bound.
- **Profiles:** "Default" is the table above and can't change; editing it starts a copy called "My keys". New,
  Rename, Reset to default and Delete work on the profile picked in the list.
- Profiles are saved in this browser on this laptop. **Export** saves the profile as a `.keys.json` file and
  **Import** loads one, to share keys between laptops.

## Hall-effect (HE) keyboards

HE keyboards know how far each key is pressed, but a browser only gets that through the keyboard's **gamepad
mode**, where the keyboard also shows up as a game controller whose keys are analog inputs. Set that up in the
keyboard's own software first:

- Wooting (60HE, 80HE, Two HE), in Wootility: bind the keys you want analog to gamepad sticks or triggers, and pick
  the gamepad mode.
  - Mac: **Classic**, the only mode there. Bind its inputs in **Edit keys** as below.
  - Windows: **Xbox** (the default). The browser sees a standard Xbox controller, so with nothing bound it simply
    joins gamepad 1 as a whole controller, analog sticks and triggers included. Binding its inputs in **Edit keys**
    works too. Its name is the generic "Xbox 360 Controller", the same as a real one, so don't also plug in an Xbox
    controller when you bind its inputs.
- Keychron HE: gamepad mode in Keychron Launcher.
- Other brands (NuPhy, DrunkDeer, MonsGeek, Razer, ...) have no gamepad mode on a Mac. Their key depth is only
  reachable over WebHID, which this page doesn't do yet.

Then in **Edit keys**, click **+** next to a control and press the HE key all the way down: the page binds the
analog input it moved (shown as 🎮 axis or button) and works out its rest and full positions by itself. The input
then gives a partial stick or trigger for a partial press, and a bar under it in the editor shows the depth live.

- **Deadzone** ignores the first part of the travel.
- **Button actuation** is how far down an analog input has to go to count as a button press.
- A keyboard with analog inputs bound no longer also joins gamepad 1 as a whole controller.

Whether the key also types its letter while in gamepad mode is set in the keyboard's software. If it does, binding
the same key both ways makes the letter press (full) and the analog depth fight: the bigger one wins, so bind it one
way only.

## Safety

- **Watchdog:** the page sends both gamepads every 20 ms. If the robot hears nothing for 250 ms (lid closed, Wi-Fi
  lost, tab in the background), both gamepads go to rest. The SDK would otherwise hold the last keys forever.
- **Focus:** keys are let go when the browser window loses focus. Cmd (for Cmd+Tab) also lets go of everything,
  because macOS stops sending key-ups while it is held.
- **One driver:** the first laptop to connect drives. Others see who drives (by IP address) and can **Take over**,
  which puts the gamepads at rest first. A laptop that goes quiet for 2 s gives up control.
- **Driver Station wins:** once a Driver Station connects, the bridge ignores everything until the robot controller
  app restarts (power cycle). It only ever acts on a robot that has not seen a Driver Station since it booted.
- Anyone on the robot's Wi-Fi can open the page, so keep the Wi-Fi password to the team.

## How it works

`TeamCode/.../keyboard/KeyboardBridge.java` starts with the robot controller app (SDK hooks `@WebHandlerRegistrar`
and `@OnCreateEventLoop`). It serves the page, and talks to it over the Robot Controller's own WebSocket (port 8081,
namespace `keyboard`).

- **Gamepads:** each input message is handed to the SDK's `EventLoopManager.gamepadEvent`, exactly where Driver
  Station gamepad packets arrive. Every OpMode reads it as `gamepad1` / `gamepad2`, edge detection (`xWasPressed()`
  …) included, with no OpMode changes. The packets carry a gamepad id that `KeyboardBridge.isKeyboard(gamepad)`
  checks; TeleOp uses it to choose rate turning.
- **OpModes:** Init / Start / Stop / Esc call the SDK's `OpModeManagerImpl` the same way the Driver Station does.
- **Telemetry:** with no Driver Station the SDK drops telemetry, so just before each OpMode's `init()` the bridge
  swaps in `MirrorTelemetry`, the SDK's own telemetry class that also hands each update to the page. HTML telemetry
  shows as it does on the Driver Station.

It relies on SDK internals (a protected field and two internal classes), checked against SDK 12.0.0. If an SDK update
breaks it, the page says it can't reach the SDK and the robot log (tag `KeyboardBridge`) has the details; driving with
a Driver Station is unaffected.

`PadState` (the message format) and `ControlLease` (who drives, the watchdog) are plain Java with unit tests:
`./gradlew :TeamCode:testDebugUnitTest`.
