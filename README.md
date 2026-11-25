# Juwei-17 controller
<img width="1405" height="1017" alt="image" src="https://github.com/user-attachments/assets/b9aac731-31bc-4239-818f-ec68da102fae" />

A small app that checks whether a Juwei-17 mount is connected over USB, and reads and changes its goto slew speed. You can use it from a graphical interface in your web browser, or from the command line.

The Juwei-17 runs [OnStep](https://github.com/hjd1964/OnStepX) firmware, which takes LX200-style text commands such as `:GVP#` over a USB serial port. This app sends those commands. ASCOM, INDI and ASIAIR drivers talk to the mount the same way.

## Setup

You need Python 3.10+ and pyserial (`python3-serial` on Debian/Ubuntu, or `pip install pyserial`).

On Linux your user must be in the `dialout` group to open serial ports. Add it once, then restart the computer:

```bash
sudo usermod -aG dialout $USER
```

The group only applies to programs started after you log in again; terminals that were already open don't get it. To use the mount before restarting, open up the port until it is unplugged:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

Run the app from this folder with `./juwei`. To get a `juwei` command on your PATH instead, run `pip install -e .`.

## Graphical interface

```bash
./juwei gui
```

This opens the controller in your web browser. Keep the terminal window open while you use it, and press Ctrl+C there to stop it. The page shows:

- whether the mount is connected and on which port, updated every 2 seconds, so plugging and unplugging shows up on its own
- firmware, tracking/park state, meridian flip mode and errors
- where the mount points (RA/Dec)
- controls to turn tracking on or off, park and unpark, set the park position, go home, and stop all motion. Park, Go home and Set park position ask for a second click, because they move the mount or overwrite a setting
- a Move pad: arrows for N, S, E and W around a Stop button, like a hand controller. Each click moves the mount one step; pick the step (1°, 5°, 10°, 30° or your own value), and the browser remembers it
- the slew speed, with preset buttons (50–200%), a slider and a percentage box, each showing the speed in deg/s it will give
- Settings: the mount's date and time (and whether its clock matches this computer) with a **Sync from this computer** button, location, horizon and overhead limits, automatic meridian flip, preferred pier side, how far past the meridian it may track, refraction compensation and the buzzer
- a **Release port** button, which closes the USB port so other software on this computer, like NINA or KStars, can use the mount. **Connect** takes it back.
- a layout switch at the top right: one column, or two columns side by side (mount, control and move on the left; slew speed and settings on the right). The browser remembers the choice. The switch only shows when the window is at least 1200 pixels wide; narrower windows always use one column.

The interface runs only on this computer (127.0.0.1) and can't be reached from other machines. Options: `--port` and `--baud` work as for the commands below, `--http-port` changes the local web port (default 8017), and `--no-browser` skips opening the browser.

To try it without the mount, run `./juwei gui --simulate`. This uses a built-in fake mount (Linux and macOS only).

## Command line

```bash
./juwei status            # connected? firmware, state, slew settings
./juwei status --json     # the same, as JSON
./juwei slew              # show slew settings only
./juwei slew --preset 2   # 1=200%, 2=150%, 3=100%, 4=67%, 5=50% of base speed
./juwei slew --percent 120
./juwei slew --us 60      # raw rate in microseconds per step (smaller is faster)
./juwei tracking on       # or: tracking off
./juwei park              # slew to the park position and stop tracking
./juwei unpark            # leave park; the firmware turns tracking on
./juwei set-park          # save the current position as the park position
./juwei home              # slew to the home position
./juwei move north 10     # slew 10 degrees north (also south, east, west)
./juwei stop              # stop all motion immediately
./juwei settings          # date/time, location, limits, meridian flip, refraction, buzzer
./juwei sync-time         # set the mount's date, time and time zone from this computer
./juwei location 47.38 8.54                   # latitude, longitude (decimal degrees, north/east positive)
./juwei limits --horizon -5 --overhead 85     # also --meridian-east N --meridian-west N (minutes)
./juwei flip --auto off --side best           # automatic meridian flip, preferred pier side
./juwei refraction on     # or off
./juwei buzzer off        # or on
./juwei ports             # list USB serial ports and which one is the mount
./juwei watch             # print a line when the mount is plugged in or unplugged
```

By default the app searches all USB serial ports and tries 9600 baud, then 115200. Use `--port /dev/ttyUSB0` (or `COM3` on Windows) and `--baud` to skip the search.

Exit codes: `0` success, `1` error, `2` mount not connected.

## What the slew numbers mean

OnStep stores the goto speed as a step rate in microseconds per step. The "base" rate is the 100% speed and is fixed when the firmware is built. At runtime you can only set 50% to 200% of base, and never faster than the hardware limit the firmware reports. The app refuses to change the rate while the mount is slewing, homing or guiding, and so does the firmware.

Any value in that range works, not only the five presets. The interface shows the resulting speed in deg/s while you move the slider, and under each preset button.

The deg/s numbers are exact when the firmware reports the mount's steps per degree (`:GXE4#`, available on OnStepX 10.20a and later). On older firmware the app estimates them from the speed the firmware reports, which is rounded to 0.1 deg/s, and marks them with "≈" (or "~" on the command line).

## Does a change survive?

- **Unplugging USB and connecting ASIAIR:** yes. The setting lives in the mount, not on the computer, and stays as long as the mount stays powered. ASIAIR could still overwrite it if it sets its own rate when it connects. To check, run `./juwei status` again after an ASIAIR session.
- **Switching the mount off:** only if the firmware was built with OnStep's `SLEW_RATE_MEMORY` option on. It is off by default in OnStepX, and it is unknown how Juwei builds theirs. To find out, set a rate, power-cycle the mount and run `./juwei status`. If the rate is back to 100%, the mount doesn't remember it.

## Parking

Parking is stored in the mount and survives switching it off. A parked mount ignores gotos and tracking until it is unparked, here or from ASIAIR. Parking fails if the mount has no park position saved; point the mount where you want it to park and use "Set park position here" (`./juwei set-park`) once. When a command fails, the app shows the firmware's reason (it asks with `:GE#`).

## Moving

Move reads where the mount points, adds the offset and sends an ordinary goto, so the mount uses the goto slew speed and the firmware still checks its horizon and axis limits. North/south change Dec; east/west turn the RA axis, where 15 degrees is one hour of RA (on the sky that is a shorter distance away from the equator). A move that would pass the pole is refused. Gotos need the mount's date, time and location; ASIAIR normally sets these when it connects. If they were never set, the mount refuses with "standby".

## Settings

All command formats were checked against the OnStepX 10.23a source, which is what the Juwei-17 runs.

- **Date and time:** "Sync" sends this computer's time zone, date and time. The mount wants the hours to *add* to local time to get UTC, so the app sends UTC+1 as `-01:00`. The time is kept only while the mount is powered, unless it has a clock chip.
- **Location:** shown and entered with east positive, like maps. The mount itself stores east as negative; the app converts.
- **Limits:** horizon −30° to +30°, overhead 60° to 90°, and how many minutes the mount may track past the meridian on each side. Saved in the mount.
- **Meridian flip, refraction, buzzer:** whether these survive a power-off depends on how Juwei built the firmware.
- Changing the location or time changes where the mount thinks it points, so do it before aligning. ASIAIR usually sends its own date, time and location when it connects.

## Caveats

- Only one program can use the USB port at a time. Disconnect KStars/INDI, NINA or other drivers first. While the graphical interface is open it holds the port; click **Release port** or stop it before connecting other software on this computer.
- Port search sends `:GVP#` to every USB serial device it finds. This is harmless for most devices, but use `--port` if other serial hardware is plugged in.
- The app has been tested against a simulated OnStep mount, not a real Juwei-17. Commands were checked against the OnStepX source. Older OnStep 4.x firmware may lack the deg/s and hardware-limit readings; the app shows the rest without them.

## Trying it without the mount

```bash
./juwei gui --simulate
```

For the command line, start a fake mount in one terminal and point the app at the port it prints:

```bash
PYTHONPATH=src python3 -m juwei_controller.simulator     # prints a port like /dev/pts/5
./juwei status --port /dev/pts/5
```

Run the tests with:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests
```
