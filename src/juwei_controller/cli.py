"""Command-line app: is the Juwei-17 connected, and what are its slew settings."""

from __future__ import annotations

import argparse
import json
import sys
import time

import serial

from .discovery import FALLBACK_BAUDS, ProbeResult, connect, find_mount, scan
from .onstep import (DEFAULT_TIMEOUT, DIRECTIONS, MIN_FACTOR, MountStatus, OnStep, OnStepError, SlewSettings,
                     format_dec, format_deg_per_s, format_ra)

EXIT_ERROR = 1
EXIT_NOT_CONNECTED = 2

PERSISTENCE_NOTE = (
    "The new rate lives in the mount, not on this computer: it stays while the mount is powered,\n"
    "also after you unplug USB and connect ASIAIR (unless ASIAIR sets its own rate).\n"
    "Whether it survives switching the mount off depends on how its firmware was built\n"
    "(OnStep's SLEW_RATE_MEMORY option): power-cycle the mount and run 'juwei status' to find out."
)


def _bauds(args: argparse.Namespace) -> tuple[int, ...]:
    return (args.baud,) if args.baud else FALLBACK_BAUDS


def _open(args: argparse.Namespace) -> tuple[OnStep | None, ProbeResult | None, list[ProbeResult]]:
    if args.port:
        mount, result = connect(args.port, args.port, _bauds(args), args.timeout)
        return mount, (result if mount else None), [result]
    return find_mount(_bauds(args), args.timeout)


def _print_not_connected(tried: list[ProbeResult]) -> None:
    if not tried:
        print("Mount: not connected (no USB serial ports found)")
        return
    print("Mount: not connected")
    for result in tried:
        print(f"  {result.device} ({result.description}): {result.error}")


def _fmt_speed(slew: SlewSettings) -> str:
    speed = ""
    if slew.current_deg_per_s is not None:
        speed = f"{format_deg_per_s(slew.current_deg_per_s, slew.speed_is_exact)} deg/s, "
    return f"{speed}{slew.factor * 100:.0f}% of base ({slew.current_us_per_step:.3f} us/step)"


def _fmt_state(status: MountStatus) -> str:
    parts = ["tracking" if status.tracking else "not tracking"]
    if status.slewing:
        parts.append("slewing")
    if status.homing:
        parts.append("homing")
    if status.guiding:
        parts.append("guiding")
    parts.append(status.park)
    if status.at_home:
        parts.append("at home")
    return ", ".join(parts)


def _print_slew(slew: SlewSettings) -> None:
    print(f"Slew speed  {_fmt_speed(slew)}")
    base = slew.base_deg_per_s
    base_speed = f", {format_deg_per_s(base, slew.speed_is_exact)} deg/s" if base is not None else ""
    print(f"  base rate     {slew.base_us_per_step:.3f} us/step (100%{base_speed}), fixed in firmware")
    if slew.fastest_us_per_step is not None:
        print(f"  fastest rate  {slew.fastest_us_per_step:.3f} us/step, hardware limit")
    print(f"  allowed       {MIN_FACTOR * 100:.0f}% to {slew.max_factor * 100:.0f}% of base")


def cmd_status(args: argparse.Namespace) -> int:
    mount, result, tried = _open(args)
    if mount is None:
        if args.json:
            print(json.dumps({"connected": False, "ports": [r.__dict__ for r in tried]}, indent=2))
        else:
            _print_not_connected(tried)
        return EXIT_NOT_CONNECTED
    with mount:
        status = mount.status()
        slew = mount.slew_settings()
        date = mount.firmware_date()
        try:
            position = mount.position()
        except OnStepError:
            position = None
    if args.json:
        print(json.dumps({
            "connected": True,
            "device": result.device,
            "description": result.description,
            "baud": result.baud,
            "product": result.product,
            "version": result.version,
            "firmware_date": date,
            "status": status.to_dict(),
            "position": position.to_dict() if position else None,
            "slew": slew.to_dict(),
        }, indent=2))
        return 0
    where = f"{result.device} ({result.description})" if result.description != result.device else result.device
    print(f"Mount       connected on {where} at {result.baud} baud")
    print(f"Firmware    {result.product} {result.version}" + (f" ({date})" if date else ""))
    print(f"State       {_fmt_state(status)}")
    if position:
        print(f"Pointing    RA {format_ra(position.ra_hours)}  Dec {format_dec(position.dec_degrees).replace('*', ' deg ')}")
    print(f"Meridian    automatic flip {'on' if status.auto_meridian_flip else 'off'}")
    print(f"Errors      {status.error}")
    print()
    _print_slew(slew)
    return 0


def cmd_slew(args: argparse.Namespace) -> int:
    mount, result, tried = _open(args)
    if mount is None:
        _print_not_connected(tried)
        return EXIT_NOT_CONNECTED
    with mount:
        before = mount.slew_settings()
        if args.preset is not None:
            after = mount.set_slew_preset(args.preset)
        elif args.percent is not None:
            after = mount.set_slew_percent(args.percent)
        elif args.us is not None:
            after = mount.set_slew_us_per_step(args.us)
        else:
            _print_slew(before)
            return 0
    print(f"Slew speed changed on {result.device}")
    print(f"  before  {_fmt_speed(before)}")
    print(f"  after   {_fmt_speed(after)}")
    if args.percent is not None and abs(after.factor * 100 - args.percent) >= 1:
        print(f"  (the mount limited {args.percent:g}% to {after.factor * 100:.0f}%)")
    print()
    print(PERSISTENCE_NOTE)
    return 0


CONTROL_DONE = {
    "park": "Parking. The mount stays parked until you unpark it, also after it is switched off.",
    "unpark": "Unparked. Tracking is on.",
    "set-park": "Park position saved.",
    "home": "Going home.",
    "stop": "Stopped all motion.",
}


def _on(text: str) -> bool:
    return text == "on"


def _set_limits(mount: OnStep, args: argparse.Namespace) -> str:
    values = (args.horizon, args.overhead, args.meridian_east, args.meridian_west)
    if all(v is None for v in values):
        raise ValueError("give at least one of --horizon, --overhead, --meridian-east, --meridian-west")
    mount.set_limits(*values)
    return "Limits saved."


def _set_flip(mount: OnStep, args: argparse.Namespace) -> str:
    if args.auto is None and args.side is None:
        raise ValueError("give --auto on|off and/or --side best|east|west")
    if args.auto is not None:
        mount.set_auto_meridian_flip(_on(args.auto))
    if args.side is not None:
        mount.set_preferred_pier_side(args.side[0].upper())
    return "Meridian flip settings saved."


def _set_location(mount: OnStep, args: argparse.Namespace) -> str:
    mount.set_location(args.latitude, args.longitude)
    return "Location saved."


def _sync_time(mount: OnStep, args: argparse.Namespace) -> str:
    mount.sync_time()
    return "Date, time and time zone set from this computer."


def _set_refraction(mount: OnStep, args: argparse.Namespace) -> str:
    mount.set_refraction(_on(args.state))
    return f"Refraction compensation {args.state}."


def _set_buzzer(mount: OnStep, args: argparse.Namespace) -> str:
    mount.set_buzzer(_on(args.state))
    return f"Buzzer {args.state}."


SETTING_COMMANDS = {
    "sync-time": _sync_time,
    "location": _set_location,
    "limits": _set_limits,
    "flip": _set_flip,
    "refraction": _set_refraction,
    "buzzer": _set_buzzer,
}


def _print_settings(settings: dict) -> None:
    site, limits = settings["site"], settings["limits"]
    offset = site["utc_offset_hours"]
    print()
    if site["date_time_ready"]:
        error = site["clock_error_s"]
        clock = "" if error is None else (", matches this computer" if abs(error) < 5
                                          else f", {abs(error):.0f} s {'ahead of' if error > 0 else 'behind'} this computer")
        print(f"Date/time   {site['local_date']} {site['local_time']} (UTC{offset:+g}){clock}")
    else:
        print("Date/time   not set (Move and gotos need it: juwei sync-time)")
    print(f"Location    latitude {site['latitude']:.4f}, longitude {site['longitude']:.4f} (east positive)")
    print(f"Limits      horizon {limits['horizon_deg']} deg, overhead {limits['overhead_deg']} deg, "
          f"past meridian east {limits['meridian_east_min']} min, west {limits['meridian_west_min']} min")
    flip = settings["auto_meridian_flip"]
    side = {"E": "east", "W": "west", "B": "best", "A": "automatic"}.get(settings["preferred_pier_side"], "unknown")
    print(f"Meridian    automatic flip {'unknown' if flip is None else 'on' if flip else 'off'}, preferred side {side}")
    print(f"Refraction  {settings['refraction']}")
    print(f"Buzzer      {'on' if settings['buzzer'] else 'off'}")


def cmd_settings(args: argparse.Namespace) -> int:
    mount, result, tried = _open(args)
    if mount is None:
        _print_not_connected(tried)
        return EXIT_NOT_CONNECTED
    with mount:
        settings = mount.settings()
    if args.json:
        print(json.dumps(settings, indent=2))
    else:
        _print_settings(settings)
    return 0


def cmd_control(args: argparse.Namespace) -> int:
    mount, result, tried = _open(args)
    if mount is None:
        _print_not_connected(tried)
        return EXIT_NOT_CONNECTED
    with mount:
        if args.command == "tracking":
            on = args.state == "on"
            mount.set_tracking(on)
            print(f"Tracking {'on' if on else 'off'}.")
            return 0
        if args.command in SETTING_COMMANDS:
            message = SETTING_COMMANDS[args.command](mount, args)
            print(message)
            _print_settings(mount.settings())
            return 0
        if args.command == "move":
            target = mount.move(args.direction, args.degrees)
            print(f"Moving {args.degrees:g} deg {args.direction} to RA {format_ra(target.ra_hours)}  "
                  f"Dec {format_dec(target.dec_degrees).replace('*', ' deg ')}. Stop it with: juwei stop")
            return 0
        action = {
            "park": mount.park,
            "unpark": mount.unpark,
            "set-park": mount.set_park_position,
            "home": mount.go_home,
            "stop": mount.stop,
        }[args.command]
        action()
    print(CONTROL_DONE[args.command])
    return 0


def cmd_ports(args: argparse.Namespace) -> int:
    results = scan(_bauds(args), args.timeout)
    if not results:
        print("No USB serial ports found. Is the mount plugged in and powered?")
        return EXIT_NOT_CONNECTED
    for r in results:
        verdict = f"OnStep mount: {r.product} {r.version} at {r.baud} baud" if r.is_mount else r.error
        print(f"{r.device}  ({r.description})\n    {verdict}")
    return 0 if any(r.is_mount for r in results) else EXIT_NOT_CONNECTED


def cmd_watch(args: argparse.Namespace) -> int:
    print("Watching for the mount, Ctrl+C to stop.")
    mount: OnStep | None = None
    announced_missing = False
    try:
        while True:
            stamp = time.strftime("%H:%M:%S")
            if mount is None:
                mount, result, _ = _open(args)
                if mount is not None:
                    print(f"{stamp}  connected on {result.device}: {result.product} {result.version}, "
                          f"slew {_fmt_speed(mount.slew_settings())}")
                    announced_missing = False
                elif not announced_missing:
                    print(f"{stamp}  not connected")
                    announced_missing = True
            else:
                try:
                    mount.status()
                except (OnStepError, serial.SerialException, OSError):
                    print(f"{stamp}  disconnected")
                    mount.close()
                    mount = None
                    announced_missing = True
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    finally:
        if mount is not None:
            mount.close()


def cmd_gui(args: argparse.Namespace) -> int:
    from . import gui

    serial_port = args.port
    simulator = None
    if args.simulate:
        from .simulator import FakeOnStep, PtyServer

        simulator = PtyServer(FakeOnStep()).start()
        serial_port = simulator.path
    service = gui.MountService(serial_port, _bauds(args), args.timeout)
    try:
        if args.http_port is None:
            server = gui.make_server_near(service, gui.DEFAULT_HTTP_PORT)
        else:
            server = gui.make_server(service, args.http_port)
    except OSError as exc:
        wanted = args.http_port if args.http_port is not None else f"{gui.DEFAULT_HTTP_PORT} or the next ports"
        print(f"Error: cannot listen on port {wanted} ({exc}). Try --http-port with another number.", file=sys.stderr)
        if simulator is not None:
            simulator.stop()
        return EXIT_ERROR
    if args.http_port is None and server.server_address[1] != gui.DEFAULT_HTTP_PORT:
        print(f"Port {gui.DEFAULT_HTTP_PORT} is in use, probably by another copy of this app; "
              f"using {server.server_address[1]} instead.")
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"Juwei-17 Controller is running at {url}" + (f" (fake mount on {serial_port})" if simulator else ""))
    print("Keep this window open while you use it. Ctrl+C to stop.")
    if not args.no_browser:
        import webbrowser

        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        service.close()
        if simulator is not None:
            simulator.stop()
    return 0


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0 or COM3 (default: search USB ports)")
    common.add_argument("--baud", type=int, help="baud rate (default: try 9600, then 115200)")
    common.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="seconds to wait for a reply")

    parser = argparse.ArgumentParser(prog="juwei", description="Check and change settings of a Juwei-17 (OnStep) mount over USB.")
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", parents=[common], help="is the mount connected, its state and slew settings")
    status.add_argument("--json", action="store_true", help="print machine-readable JSON")
    status.set_defaults(func=cmd_status)

    slew = sub.add_parser("slew", parents=[common], help="show or change the goto slew speed")
    choice = slew.add_mutually_exclusive_group()
    choice.add_argument("--preset", type=int, choices=range(1, 6), metavar="1-5",
                        help="OnStep preset: 1=200%%, 2=150%%, 3=100%%, 4=67%%, 5=50%% of base")
    choice.add_argument("--percent", type=float, help="speed as %% of base, 50 to 200")
    choice.add_argument("--us", type=float, help="rate in microseconds per step (smaller is faster)")
    slew.set_defaults(func=cmd_slew)

    for name, text in (("park", "move to the park position and stop tracking"),
                       ("unpark", "leave the park position and start tracking"),
                       ("set-park", "save the current position as the park position"),
                       ("home", "move to the home position"),
                       ("stop", "stop all motion immediately")):
        sub.add_parser(name, parents=[common], help=text).set_defaults(func=cmd_control)
    tracking = sub.add_parser("tracking", parents=[common], help="turn tracking on or off")
    tracking.add_argument("state", choices=["on", "off"])
    tracking.set_defaults(func=cmd_control)
    move = sub.add_parser("move", parents=[common], help="slew a number of degrees north, south, east or west")
    move.add_argument("direction", choices=DIRECTIONS)
    move.add_argument("degrees", type=float, help="how far, e.g. 10 (east/west turns the RA axis)")
    move.set_defaults(func=cmd_control)

    settings = sub.add_parser("settings", parents=[common], help="show date/time, location, limits, flip, refraction, buzzer")
    settings.add_argument("--json", action="store_true", help="print machine-readable JSON")
    settings.set_defaults(func=cmd_settings)
    sub.add_parser("sync-time", parents=[common], help="set the mount's date, time and time zone from this computer"
                   ).set_defaults(func=cmd_control)
    location = sub.add_parser("location", parents=[common], help="set latitude and longitude (decimal degrees, east positive)")
    location.add_argument("latitude", type=float)
    location.add_argument("longitude", type=float)
    location.set_defaults(func=cmd_control)
    limits = sub.add_parser("limits", parents=[common], help="set horizon/overhead/meridian limits")
    limits.add_argument("--horizon", type=int, help="lowest altitude for gotos, -30 to 30 deg")
    limits.add_argument("--overhead", type=int, help="highest altitude for gotos, 60 to 90 deg")
    limits.add_argument("--meridian-east", type=int, help="minutes the mount may track past the meridian, east side")
    limits.add_argument("--meridian-west", type=int, help="minutes the mount may track past the meridian, west side")
    limits.set_defaults(func=cmd_control)
    flip = sub.add_parser("flip", parents=[common], help="automatic meridian flip and preferred pier side")
    flip.add_argument("--auto", choices=["on", "off"])
    flip.add_argument("--side", choices=["best", "east", "west"])
    flip.set_defaults(func=cmd_control)
    for name, text in (("refraction", "refraction compensation on or off"), ("buzzer", "buzzer on or off")):
        toggle = sub.add_parser(name, parents=[common], help=text)
        toggle.add_argument("state", choices=["on", "off"])
        toggle.set_defaults(func=cmd_control)

    ports = sub.add_parser("ports", parents=[common], help="list USB serial ports and which one is the mount")
    ports.set_defaults(func=cmd_ports)

    watch = sub.add_parser("watch", parents=[common], help="report when the mount is connected or unplugged")
    watch.add_argument("--interval", type=float, default=2.0, help="seconds between checks")
    watch.set_defaults(func=cmd_watch)

    gui_cmd = sub.add_parser("gui", parents=[common], help="open the graphical interface in your web browser")
    gui_cmd.add_argument("--http-port", type=int,
                         help="local port for the interface (default: 8017, or the next free one)")
    gui_cmd.add_argument("--no-browser", action="store_true", help="don't open the browser automatically")
    gui_cmd.add_argument("--simulate", action="store_true", help="use a built-in fake mount instead of USB (Linux/macOS)")
    gui_cmd.set_defaults(func=cmd_gui)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (OnStepError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return EXIT_ERROR
    except (serial.SerialException, OSError) as exc:
        print(f"Error: lost connection to the mount ({exc})", file=sys.stderr)
        return EXIT_ERROR
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
