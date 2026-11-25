"""A fake OnStep mount on a pseudo-terminal, for trying the app and for tests without real hardware.

Run ``python -m juwei_controller.simulator`` and point the app at the printed port.
It answers only the commands this app uses, following OnStepX's replies and limits.
Park and home "move" for a few seconds so the app's moving state can be seen.
"""

from __future__ import annotations

import argparse
import os
import re
import select
import threading
import time
import tty
from datetime import datetime, timedelta, timezone

# OnStepX CommandError codes used here.
CE_PARKED = 9
CE_NOT_PARKED = 11
CE_NO_PARK_POSITION_SET = 12
CE_SLEW_IN_MOTION = 22


class FakeOnStep:
    def __init__(self, steps_per_degree: float = 12800.0, base_deg_per_s: float = 1.0,
                 fastest_us_per_step: float = 16.0, version: str = "10.24c", legacy: bool = False,
                 reported_steps_per_degree: float | None = None, park_saved: bool = True,
                 motion_seconds: float = 3.0):
        self.steps_per_degree = steps_per_degree
        # What :GXE4# answers; set it differently from steps_per_degree to mimic a firmware fork.
        self.reported_steps_per_degree = reported_steps_per_degree or steps_per_degree
        self.base_us = 1_000_000.0 / (steps_per_degree * base_deg_per_s)
        self.current_us = self.base_us
        self.fastest_us = fastest_us_per_step
        self.version = version
        self.legacy = legacy  # older firmware: no :GX97#, :GX99# or :GXE4#
        self.tracking = True
        self.slewing = False  # set by tests to mimic a goto started elsewhere
        self.park_saved = park_saved
        self.park = "unparked"  # unparked, parking, parked
        self.at_home = False
        self.homing = False
        self.motion_seconds = motion_seconds
        self.ra_hours = 5.5
        self.dec_degrees = 20.0
        self.time_ready = True  # False mimics a mount whose date/time was never set
        self.date_ready = True
        self.latitude = 47.0
        self.longitude_west = -8.5  # as the mount stores it: east is negative
        self.to_utc_hours = -1.0  # hours added to local time to get UTC (-1 means UTC+1)
        self._utc_base = datetime.now(timezone.utc)
        self._utc_set_at = time.monotonic()
        self.horizon = -10
        self.overhead = 85
        self.meridian_east = 15
        self.meridian_west = 15
        self.auto_flip = True
        self.pier_side = "B"
        self.refraction = False
        self.buzzer = True
        self.horizon_dec = -40.0  # crude horizon: targets south of this Dec are "below the horizon"
        self._target: tuple[float | None, float | None] = (None, None)
        self._goto_target: tuple[float, float] | None = None
        self.error_code = 0
        self.last_command_error = 0
        self.commands: list[str] = []
        self._motion_ends = 0.0
        self._buffer = ""
        self._lock = threading.Lock()

    def feed(self, data: bytes) -> bytes:
        """Take bytes written by the client and return the bytes the mount would send back."""
        out = b""
        with self._lock:
            for ch in data.decode("ascii", errors="replace"):
                self._buffer += ch
                if ch == "#":
                    command, self._buffer = self._buffer, ""
                    if command.startswith(":"):
                        self.commands.append(command)
                        out += self._handle(command[1:-1])
        return out

    # --- motion -----------------------------------------------------------------------------

    def _update_motion(self) -> None:
        if self._motion_ends and time.monotonic() >= self._motion_ends:
            self._motion_ends = 0.0
            if self.park == "parking":
                self.park = "parked"
            if self.homing:
                self.homing = False
                self.at_home = True
            if self._goto_target:
                self.ra_hours, self.dec_degrees = self._goto_target
                self._goto_target = None
                self.tracking = True

    def _moving(self) -> bool:
        self._update_motion()
        return self.slewing or self._motion_ends > 0

    def _start_motion(self) -> None:
        self.tracking = False
        self.at_home = False
        self._motion_ends = time.monotonic() + self.motion_seconds
        self._update_motion()

    # --- commands ---------------------------------------------------------------------------

    def _numeric(self, error: int) -> bytes:
        self.last_command_error = error
        return b"0" if error else b"1"

    def _clamp(self, us: float) -> float:
        us = min(max(us, self.base_us / 2), self.base_us * 2)
        return max(us, self.fastest_us)

    def _handle(self, body: str) -> bytes:
        if body == "":
            return b""  # ":#" just clears the command buffer
        if body == "GE":
            return f"{self.last_command_error:02d}#".encode()
        self.last_command_error = 0
        if body == "GVP":
            return b"On-Step#"
        if body == "GVN":
            return f"{self.version}#".encode()
        if body == "GVD":
            return b"Nov 25 2026#"
        if body == "GU":
            return self._status().encode() + b"#"
        if body == "GX92":
            return f"{self.current_us:.3f}#".encode()
        if body == "GX93":
            return f"{self.base_us:.3f}#".encode()
        if body == "GX97" and not self.legacy:
            return f"{1_000_000.0 / self.current_us / self.steps_per_degree:.1f}#".encode()
        if body == "GX99" and not self.legacy:
            return f"{self.fastest_us:.3f}#".encode()
        if body == "GXE4" and not self.legacy:
            return f"{round(self.reported_steps_per_degree)}#".encode()
        if body.startswith("SX92,"):
            if self._moving():
                return self._numeric(CE_SLEW_IN_MOTION)
            self.current_us = self._clamp(float(body[5:]))
            return self._numeric(0)
        if body.startswith("SX93,"):
            if not self._moving():
                factor = {"1": 0.5, "2": 1 / 1.5, "3": 1.0, "4": 1.5, "5": 2.0}.get(body[5:], 1.0)
                self.current_us = max(self.base_us * factor, self.fastest_us)
            return b""
        if body == "hP":
            if not self.park_saved:
                return self._numeric(CE_NO_PARK_POSITION_SET)
            if self.park == "parked":
                return self._numeric(0)
            if self._moving():
                return self._numeric(CE_SLEW_IN_MOTION)
            self.park = "parking"
            self._start_motion()
            return self._numeric(0)
        if body == "hR":
            if not self.park_saved:
                return self._numeric(CE_NO_PARK_POSITION_SET)
            if self.park != "parked":
                return self._numeric(CE_NOT_PARKED)
            self.park = "unparked"
            self.tracking = True
            return self._numeric(0)
        if body == "hQ":
            if self.park == "parked":
                return self._numeric(CE_PARKED)
            if self._moving():
                return self._numeric(CE_SLEW_IN_MOTION)
            self.park_saved = True
            return self._numeric(0)
        if body == "hC":
            if self.park == "parked":
                self.last_command_error = CE_PARKED
            elif self._moving():
                self.last_command_error = CE_SLEW_IN_MOTION
            else:
                self.homing = True
                self._start_motion()
            return b""
        if body == "Te":
            if self.park == "parked":
                return self._numeric(CE_PARKED)
            self.tracking = True
            return self._numeric(0)
        if body == "Td":
            self.tracking = False
            return self._numeric(0)
        if body == "GRa":
            return self._hms(self.ra_hours).encode() + b"#"
        if body == "GDe":
            return self._dms(self.dec_degrees).encode() + b"#"
        if body.startswith("Sr") or body.startswith("Sd"):
            try:
                value = self._parse(body[2:])
            except ValueError:
                return self._numeric(4)  # CE_PARAM_RANGE
            if body[1] == "r":
                self._target = (value, self._target[1])
            else:
                self._target = (self._target[0], value)
            return self._numeric(0)
        if body == "MS":
            ra, dec = self._target
            if ra is None or dec is None:
                return b"9"
            if not self.time_ready:
                return b"3"
            if self.park == "parked":
                return b"4"
            if self._moving():
                return b"5"
            if dec < self.horizon_dec:
                return b"1"
            self._goto_target = (ra, dec)
            self._start_motion()
            return b"0"
        reply = self._handle_settings(body)
        if reply is not None:
            return reply
        if body == "Q":
            self._motion_ends = 0.0
            self._goto_target = None
            self.slewing = False
            self.homing = False
            if self.park == "parking":
                self.park = "unparked"
            return b""
        self.last_command_error = 2  # CE_CMD_UNKNOWN
        return b"0"

    # --- site, time and settings -------------------------------------------------------------

    def _local_now(self) -> datetime:
        utc = self._utc_base + timedelta(seconds=time.monotonic() - self._utc_set_at)
        return utc - timedelta(hours=self.to_utc_hours)

    def _set_local(self, local: datetime) -> None:
        self._utc_base = local + timedelta(hours=self.to_utc_hours)
        self._utc_set_at = time.monotonic()

    def _handle_settings(self, body: str) -> bytes | None:
        """Site, time, limits, flip, refraction and buzzer commands; None if ``body`` is none of them."""
        local = self._local_now()
        if body == "GtH":
            return self._dms(self.latitude).encode() + b"#"
        if body == "GgH":
            return self._dms(self.longitude_west).encode() + b"#"
        if body == "GG":
            minutes = round(self.to_utc_hours * 60)
            return f"{'-' if minutes < 0 else '+'}{abs(minutes) // 60:02d}:{abs(minutes) % 60:02d}#".encode()
        if body == "GC":
            return f"{local:%m/%d/%y}#".encode()
        if body == "GL":
            return f"{local:%H:%M:%S}#".encode()
        if body == "GX89":
            return b"0" if self.date_ready and self.time_ready else b"1"
        if body.startswith("St"):
            if not re.fullmatch(r"[+-]\d{2}\*\d{2}(:\d{2})?", body[2:]):
                return self._numeric(5)  # CE_PARAM_FORM
            self.latitude = self._parse(body[2:] if body.count(":") else body[2:] + ":00")
            return self._numeric(0)
        if body.startswith("Sg"):
            if not re.fullmatch(r"[+-]?\d{3}\*\d{2}(:\d{2})?", body[2:]):
                return self._numeric(5)
            self.longitude_west = self._parse(body[2:] if body.count(":") else body[2:] + ":00")
            return self._numeric(0)
        if body.startswith("SG"):
            match = re.fullmatch(r"([+-]?)(\d{1,2})(?::(00|30|45))?", body[2:])
            if not match:
                return self._numeric(5)
            hours = int(match[2]) + int(match[3] or 0) / 60
            self.to_utc_hours = -hours if match[1] == "-" else hours
            return self._numeric(0)
        if body.startswith("SC"):
            if not re.fullmatch(r"\d{2}/\d{2}/\d{2}", body[2:]):
                return self._numeric(5)
            month, day, year = (int(part) for part in body[2:].split("/"))
            self._set_local(local.replace(year=2000 + year, month=month, day=day))
            self.date_ready = True
            return self._numeric(0)
        if body.startswith("SL"):
            if not re.fullmatch(r"\d{2}:\d{2}:\d{2}", body[2:]):
                return self._numeric(5)
            hour, minute, second = (int(part) for part in body[2:].split(":"))
            self._set_local(local.replace(hour=hour, minute=minute, second=second))
            self.time_ready = True
            return self._numeric(0)
        if body == "Gh":
            return f"{self.horizon:+d}*#".encode()
        if body == "Go":
            return f"{self.overhead:02d}*#".encode()
        if body == "GXE9":
            return f"{self.meridian_east}#".encode()
        if body == "GXEA":
            return f"{self.meridian_west}#".encode()
        limits = {"Sh": ("horizon", -30, 30), "So": ("overhead", 60, 90),
                  "SXE9,": ("meridian_east", -1440, 1440), "SXEA,": ("meridian_west", -1440, 1440)}
        for prefix, (attribute, low, high) in limits.items():
            if body.startswith(prefix):
                text = body[len(prefix):]
                if not re.fullmatch(r"[+-]?\d+", text):
                    return self._numeric(5)
                if not low <= int(text) <= high:
                    return self._numeric(4)  # CE_PARAM_RANGE
                setattr(self, attribute, int(text))
                return self._numeric(0)
        if body == "GX95":
            return f"{int(self.auto_flip)}#".encode()
        if body == "GX96":
            return f"{self.pier_side}#".encode()
        if body in ("SX95,0", "SX95,1"):
            self.auto_flip = body.endswith("1")
            return self._numeric(0)
        if body.startswith("SX96,"):
            if body[5:] not in ("E", "W", "B"):
                return self._numeric(4)
            self.pier_side = body[5:]
            return self._numeric(0)
        if body in ("SX97,0", "SX97,1"):
            self.buzzer = body.endswith("1")
            return self._numeric(0)
        if body in ("Tr", "Tn"):
            self.refraction = body == "Tr"
            return self._numeric(0)
        return None

    @staticmethod
    def _hms(hours: float) -> str:
        total = hours * 3600
        return f"{int(total // 3600):02d}:{int(total % 3600 // 60):02d}:{total % 60:07.4f}"

    @staticmethod
    def _dms(degrees: float) -> str:
        total = abs(degrees) * 3600
        sign = "-" if degrees < 0 else "+"
        return f"{sign}{int(total // 3600):02d}*{int(total % 3600 // 60):02d}:{total % 60:06.3f}"

    @staticmethod
    def _parse(text: str) -> float:
        """'05:34:31' or '+22*00:52' -> decimal; raises ValueError on garbage, like the firmware rejecting it."""
        sign = -1 if text.startswith("-") else 1
        parts = text.lstrip("+-").replace("*", ":").split(":")
        if len(parts) != 3:
            raise ValueError(text)
        h, m, s = (float(p) for p in parts)
        return sign * (h + m / 60 + s / 3600)

    def _status(self) -> str:
        moving = self._moving()
        s = ""
        if not self.tracking:
            s += "n"
        if not moving:
            s += "N"
        s += {"unparked": "p", "parking": "I", "parked": "P"}[self.park]
        if self.at_home:
            s += "H"
        if self.homing:
            s += "h"
        if self.refraction:
            s += "rs"
        if self.buzzer:
            s += "z"
        if self.auto_flip:
            s += "a"
        s += "E"
        s += "T"
        return s + "15" + chr(ord("0") + self.error_code)


class PtyServer:
    """Serve a FakeOnStep on a pseudo-terminal such as /dev/pts/7."""

    def __init__(self, mount: FakeOnStep):
        self.mount = mount
        self._master, self._slave = os.openpty()
        tty.setraw(self._slave)
        self.path = os.ttyname(self._slave)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            ready, _, _ = select.select([self._master], [], [], 0.05)
            if not ready:
                continue
            try:
                data = os.read(self._master, 1024)
            except OSError:
                continue
            reply = self.mount.feed(data)
            if reply:
                os.write(self._master, reply)

    def start(self) -> PtyServer:
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join()
        os.close(self._master)
        os.close(self._slave)

    def __enter__(self) -> PtyServer:
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a fake OnStep mount on a pseudo-terminal.")
    parser.add_argument("--legacy", action="store_true", help="behave like older firmware without :GX97#/:GX99#")
    parser.add_argument("--fastest-us", type=float, default=16.0, help="hardware speed limit in us/step")
    parser.add_argument("--base-deg", type=float, default=1.0, help="base (100%%) slew speed in deg/s")
    args = parser.parse_args()
    fake = FakeOnStep(base_deg_per_s=args.base_deg, fastest_us_per_step=args.fastest_us, legacy=args.legacy)
    with PtyServer(fake) as server:
        print(f"Fake mount on {server.path}")
        print(f"Try:  juwei status --port {server.path}")
        print("Ctrl+C to stop.")
        try:
            threading.Event().wait()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
