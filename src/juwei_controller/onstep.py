"""Talk to an OnStep / OnStepX controller (the firmware inside the Juwei-17) over a serial port.

OnStep speaks an extended LX200 protocol: commands look like ``:GVP#``.
String replies end with ``#``; set commands that report success reply a bare
``1`` or ``0``; a few set commands reply nothing at all; an unknown command
gets a bare ``0``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone

import serial

DEFAULT_BAUD = 9600
DEFAULT_TIMEOUT = 1.0
PRODUCT_PREFIX = "On-Step"

# :SX93,n# presets, as a multiple of the firmware's base slew speed.
SLEW_PRESETS: dict[int, float] = {1: 2.0, 2: 1.5, 3: 1.0, 4: 2 / 3, 5: 0.5}
MIN_FACTOR = 0.5
MAX_FACTOR = 2.0

ERROR_CODES = {
    0: "none",
    1: "motor fault",
    2: "below minimum altitude",
    3: "limit switch triggered",
    4: "Dec limit exceeded",
    5: "Azm limit exceeded",
    6: "under-pole limit exceeded",
    7: "meridian limit exceeded",
    8: "sync failed",
    9: "park failed",
    10: "goto sync failed",
    11: "unspecified error",
    12: "above maximum altitude",
    13: "weather sensor init failed",
    14: "time/location init failed",
    15: "settings memory init failed",
}


# OnStepX command error codes, as returned by :GE# (src/lib/commands/CommandErrors.h).
COMMAND_ERRORS = {
    2: "the firmware doesn't know this command",
    8: "the mount is not parked or at home",
    9: "the mount is parked (unpark it first)",
    10: "parking failed",
    11: "the mount is not parked",
    12: "no park position has been set (use 'set park position' first)",
    17: "the mount is in standby (its date and time may not be set yet)",
    18: "the mount is parked (unpark it first)",
    19: "the mount is already slewing",
    20: "outside the mount's limits",
    21: "motor or hardware fault",
    22: "the mount is moving (stop it first)",
    23: "the mount refused (unspecified error)",
}
_NO_ERROR_CODES = {0, 25}  # CE_NONE, CE_1

# Replies to :MS# (goto), '1'..'9'.
GOTO_ERRORS = {
    1: "the target is below the horizon limit",
    2: "the target is above the overhead limit",
    3: "the mount is in standby (its date and time may not be set yet)",
    4: "the mount is parked (unpark it first)",
    5: "a goto is already in progress",
    6: "the target is outside the mount's limits",
    7: "motor or hardware fault",
    8: "the mount is already moving",
    9: "the mount refused (unspecified error)",
}
DIRECTIONS = ("north", "south", "east", "west")


@dataclass(frozen=True)
class Position:
    ra_hours: float
    dec_degrees: float

    def to_dict(self) -> dict:
        return {"ra_hours": self.ra_hours, "dec_degrees": self.dec_degrees,
                "ra": format_ra(self.ra_hours), "dec": format_dec(self.dec_degrees)}

    def moved(self, direction: str, degrees: float) -> Position:
        """The position ``degrees`` away along one axis. East/west turns the RA axis (15 deg = 1 hour of RA)."""
        if direction not in DIRECTIONS:
            raise ValueError(f"direction must be one of: {', '.join(DIRECTIONS)}")
        if not 0 < degrees <= 180:
            raise ValueError("degrees must be more than 0 and at most 180")
        if direction in ("north", "south"):
            dec = self.dec_degrees + (degrees if direction == "north" else -degrees)
            if abs(dec) > 90:
                raise ValueError(f"moving {degrees:g} deg {direction} would go past the pole "
                                 f"(Dec would be {dec:+.1f} deg)")
            return Position(self.ra_hours, dec)
        hours = degrees / 15 * (1 if direction == "east" else -1)
        return Position((self.ra_hours + hours) % 24, self.dec_degrees)


def _sexagesimal(text: str) -> float:
    """'05:34:31.94', '+22*00:52.2', "-08\xdf12'05" -> decimal value."""
    text = text.strip()
    sign = -1 if text.startswith("-") else 1
    parts = re.findall(r"\d+(?:\.\d+)?", text)
    if not parts:
        raise OnStepError(f"can't read coordinate {text!r}")
    value = 0.0
    for scale, part in zip((1, 60, 3600), parts):
        value += float(part) / scale
    return sign * value


def format_ra(hours: float) -> str:
    seconds = round(hours * 3600) % (24 * 3600)
    return f"{seconds // 3600:02d}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"


def format_dec(degrees: float) -> str:
    """LX200 style, e.g. '+22*00:52' (the '*' is the degree sign)."""
    arcsec = round(abs(degrees) * 3600)
    sign = "-" if degrees < 0 else "+"
    return f"{sign}{arcsec // 3600:02d}*{arcsec % 3600 // 60:02d}:{arcsec % 60:02d}"


class OnStepError(Exception):
    """The controller did not answer, or refused a command."""


class UnsupportedCommand(OnStepError):
    """The firmware does not know this command (older OnStep versions)."""


def format_deg_per_s(value: float, exact: bool = True) -> str:
    """1.5 -> '1.5', 0.6667 -> '0.67'; estimates get one decimal and a '~'."""
    if not exact:
        return f"~{value:.1f}"
    text = f"{value:.2f}"
    return text[:-1] if text.endswith("0") else text


@dataclass(frozen=True)
class SlewSettings:
    current_us_per_step: float
    base_us_per_step: float
    fastest_us_per_step: float | None
    current_deg_per_s: float | None
    # Known on OnStepX (:GXE4#). Without it, deg/s comes from :GX97#, which the firmware rounds to 0.1.
    steps_per_degree: float | None = None

    @property
    def speed_is_exact(self) -> bool:
        return self.steps_per_degree is not None

    def deg_per_s_at(self, us_per_step: float) -> float | None:
        """Slew speed in deg/s for a given rate in us/step."""
        if self.steps_per_degree is not None:
            return 1_000_000.0 / (us_per_step * self.steps_per_degree)
        if self.current_deg_per_s is not None:
            return self.current_deg_per_s * self.current_us_per_step / us_per_step
        return None

    @property
    def base_deg_per_s(self) -> float | None:
        return self.deg_per_s_at(self.base_us_per_step)

    @property
    def factor(self) -> float:
        """Current slew speed as a multiple of the base speed (1.0 means 100%)."""
        return self.base_us_per_step / self.current_us_per_step

    @property
    def max_factor(self) -> float:
        """Fastest speed the firmware will accept, as a multiple of the base speed."""
        if self.fastest_us_per_step is None:
            return MAX_FACTOR
        return min(MAX_FACTOR, self.base_us_per_step / self.fastest_us_per_step)

    def to_dict(self) -> dict:
        return {
            "current_us_per_step": self.current_us_per_step,
            "base_us_per_step": self.base_us_per_step,
            "fastest_us_per_step": self.fastest_us_per_step,
            "current_deg_per_s": self.current_deg_per_s,
            "base_deg_per_s": self.base_deg_per_s,
            "steps_per_degree": self.steps_per_degree,
            "speed_is_exact": self.speed_is_exact,
            "percent_of_base": round(self.factor * 100, 1),
            "allowed_percent": [MIN_FACTOR * 100, round(self.max_factor * 100, 1)],
        }


@dataclass(frozen=True)
class MountStatus:
    raw: str
    tracking: bool
    slewing: bool
    park: str
    at_home: bool
    homing: bool
    guiding: bool
    auto_meridian_flip: bool
    error_code: int | None
    refraction: str = "off"  # "off", "refraction" or "full model"
    buzzer: bool = False

    @property
    def moving(self) -> bool:
        return self.slewing or self.homing or self.guiding

    @property
    def error(self) -> str:
        if self.error_code is None:
            return "unknown"
        return ERROR_CODES.get(self.error_code, f"error {self.error_code}")

    def to_dict(self) -> dict:
        return {
            "tracking": self.tracking,
            "slewing": self.slewing,
            "park": self.park,
            "at_home": self.at_home,
            "homing": self.homing,
            "guiding": self.guiding,
            "auto_meridian_flip": self.auto_meridian_flip,
            "refraction": self.refraction,
            "buzzer": self.buzzer,
            "error_code": self.error_code,
            "error": self.error,
            "raw": self.raw,
        }


def parse_status(raw: str) -> MountStatus:
    """Parse the reply to ``:GU#`` (without the trailing ``#``)."""
    if "P" in raw:
        park = "parked"
    elif "I" in raw:
        park = "parking"
    elif "F" in raw:
        park = "park failed"
    elif "p" in raw:
        park = "unparked"
    else:
        park = "unknown"
    # The reply ends with three digit characters: pulse-guide rate, guide rate, error code ('0' + code).
    error_code = None
    if len(raw) >= 3 and raw[-3].isdigit() and raw[-2].isdigit() and raw[-1] >= "0":
        error_code = ord(raw[-1]) - ord("0")
    return MountStatus(
        raw=raw,
        tracking="n" not in raw,
        slewing="N" not in raw,
        park=park,
        at_home="H" in raw,
        homing="h" in raw,
        guiding="g" in raw or "G" in raw,
        auto_meridian_flip="a" in raw,
        error_code=error_code,
        refraction="full model" if "t" in raw else "refraction" if "r" in raw else "off",
        buzzer="z" in raw,
    )


@dataclass(frozen=True)
class Site:
    latitude: float  # degrees, north positive
    longitude: float  # degrees, EAST positive (the mount itself stores east as negative)
    utc_offset_hours: float  # local time = UTC + this, e.g. +1 for central Europe in winter
    local_date: str  # as the mount reports it, MM/DD/YY
    local_time: str  # HH:MM:SS
    date_time_ready: bool
    clock_error_s: float | None  # mount clock minus this computer's clock; None if it can't be read

    def to_dict(self) -> dict:
        return {
            "latitude": self.latitude, "longitude": self.longitude, "utc_offset_hours": self.utc_offset_hours,
            "local_date": self.local_date, "local_time": self.local_time,
            "date_time_ready": self.date_time_ready, "clock_error_s": self.clock_error_s,
        }


@dataclass(frozen=True)
class Limits:
    horizon_deg: int  # lowest altitude for gotos, -30..30
    overhead_deg: int  # highest altitude for gotos, 60..90
    meridian_east_min: int | None  # how far past the meridian the mount may go, in minutes of time
    meridian_west_min: int | None

    def to_dict(self) -> dict:
        return {"horizon_deg": self.horizon_deg, "overhead_deg": self.overhead_deg,
                "meridian_east_min": self.meridian_east_min, "meridian_west_min": self.meridian_west_min}


PIER_SIDES = {"E": "east", "W": "west", "B": "best", "A": "automatic"}


def _format_signed_dms(degrees: float, degree_digits: int) -> str:
    """+DD*MM:SS for latitude (2 digits) or +DDD*MM:SS for longitude (3 digits), as the firmware's parser wants."""
    arcsec = round(abs(degrees) * 3600)
    sign = "-" if degrees < 0 else "+"
    return f"{sign}{arcsec // 3600:0{degree_digits}d}*{arcsec % 3600 // 60:02d}:{arcsec % 60:02d}"


def format_utc_offset_for_mount(utc_offset_hours: float) -> str:
    """The mount wants the hours to ADD to local time to get UTC, so UTC+1 is sent as '-01:00'."""
    minutes = round(-utc_offset_hours * 60)
    if abs(minutes) % 60 not in (0, 30, 45):
        raise ValueError("the mount only accepts time zones on whole, half or three-quarter hours")
    sign = "-" if minutes < 0 else "+"
    return f"{sign}{abs(minutes) // 60:02d}:{abs(minutes) % 60:02d}"


class OnStep:
    """A connection to one OnStep controller."""

    def __init__(self, port: serial.Serial):
        self._port = port
        self._steps_per_degree: float | None = None
        self._steps_per_degree_read = False

    @classmethod
    def open(cls, device: str, baud: int = DEFAULT_BAUD, timeout: float = DEFAULT_TIMEOUT) -> OnStep:
        port = serial.Serial()
        port.port = device
        port.baudrate = baud
        port.timeout = timeout
        port.write_timeout = timeout
        port.exclusive = True
        # ESP32 boards reset when DTR/RTS toggle; keep both released so connecting doesn't reboot the mount.
        port.dtr = False
        port.rts = False
        port.open()
        port.reset_input_buffer()
        return cls(port)

    @property
    def device(self) -> str:
        return self._port.port

    def close(self) -> None:
        self._port.close()

    def __enter__(self) -> OnStep:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _send(self, command: str) -> None:
        self._port.reset_input_buffer()
        self._port.write(command.encode("ascii"))
        self._port.flush()

    def query(self, command: str) -> str:
        """Send a command that answers with a ``#``-terminated string."""
        self._send(command)
        data = self._port.read_until(b"#")
        if data.endswith(b"#"):
            return data[:-1].decode("ascii", errors="replace")
        if data == b"0":
            raise UnsupportedCommand(f"the mount firmware does not support {command}")
        if not data:
            raise OnStepError(f"no reply to {command}")
        raise OnStepError(f"incomplete reply to {command}: {data!r}")

    def query_bool(self, command: str) -> bool:
        """Send a command that answers with a bare ``1`` (accepted) or ``0`` (refused)."""
        self._send(command)
        data = self._port.read(1)
        if data == b"1":
            return True
        if data == b"0":
            return False
        raise OnStepError(f"unexpected reply to {command}: {data!r}")

    def _query_optional(self, command: str) -> str | None:
        try:
            return self.query(command)
        except UnsupportedCommand:
            return None

    def product(self) -> str:
        return self.query(":GVP#")

    def firmware_version(self) -> str:
        return self.query(":GVN#")

    def firmware_date(self) -> str | None:
        return self._query_optional(":GVD#")

    def status(self) -> MountStatus:
        return parse_status(self.query(":GU#"))

    def steps_per_degree(self) -> float | None:
        """RA axis steps per degree, or None if the firmware doesn't report it. Read once per connection."""
        if not self._steps_per_degree_read:
            # :GXE4# is the running value, rounded to a whole step (OnStepX 10.20a and later).
            # :GXA1,1# looks similar but answers "0" with the factory axis settings, and doesn't exist in 10.23a.
            reply = self._query_optional(":GXE4#")
            try:
                value = float(reply) if reply else 0.0
            except ValueError:
                value = 0.0
            self._steps_per_degree = value if value > 0 else None
            self._steps_per_degree_read = True
        return self._steps_per_degree

    def slew_settings(self) -> SlewSettings:
        current = float(self.query(":GX92#"))
        base = float(self.query(":GX93#"))
        fastest = self._query_optional(":GX99#")
        reported = self._query_optional(":GX97#")
        reported_deg = float(reported) if reported is not None else None
        steps = self.steps_per_degree()
        exact_deg = 1_000_000.0 / (current * steps) if steps else None
        if exact_deg is not None and reported_deg is not None and abs(exact_deg - reported_deg) > 0.051:
            # Doesn't match the firmware's own (rounded) speed, so this build means something else by it.
            steps, exact_deg = None, None
        return SlewSettings(
            current_us_per_step=current,
            base_us_per_step=base,
            fastest_us_per_step=float(fastest) if fastest is not None else None,
            current_deg_per_s=exact_deg if exact_deg is not None else reported_deg,
            steps_per_degree=steps,
        )

    def _require_stationary(self) -> None:
        status = self.status()
        if status.moving:
            raise OnStepError("the mount is moving (slewing, homing or guiding); stop it before changing the slew rate")

    def set_slew_preset(self, preset: int) -> SlewSettings:
        """Pick one of OnStep's five presets: 1 = 200%, 2 = 150%, 3 = 100%, 4 = 67%, 5 = 50%."""
        if preset not in SLEW_PRESETS:
            raise ValueError(f"preset must be 1-5, got {preset}")
        self._require_stationary()
        before = self.slew_settings()
        self._send(f":SX93,{preset}#")  # this command never replies
        after = self.slew_settings()
        # Older OnStep versions scale some presets slightly differently, so only catch "nothing happened".
        expected_us = max(before.base_us_per_step / SLEW_PRESETS[preset], before.fastest_us_per_step or 0.0)
        unchanged = abs(after.current_us_per_step - before.current_us_per_step) < 0.001
        if unchanged and abs(expected_us - before.current_us_per_step) > 0.01 * expected_us:
            raise OnStepError(f"the mount did not apply preset {preset}")
        return after

    def set_slew_us_per_step(self, us_per_step: float) -> SlewSettings:
        """Set the slew rate directly. The firmware limits it to 50%-200% of base and the hardware maximum."""
        if us_per_step <= 0:
            raise ValueError("us_per_step must be positive")
        self._require_stationary()
        if not self.query_bool(f":SX92,{us_per_step:.3f}#"):
            raise OnStepError("the mount refused the new slew rate")
        return self.slew_settings()

    def last_error(self) -> str | None:
        """Why the previous command failed, from :GE#. None if it didn't fail or the firmware can't say."""
        reply = self._query_optional(":GE#")
        try:
            code = int(reply) if reply is not None else 0
        except ValueError:
            return None
        if code in _NO_ERROR_CODES:
            return None
        return COMMAND_ERRORS.get(code, f"error code {code}")

    def _run(self, command: str, what: str) -> None:
        """Send a command that replies 1 or 0, and raise with the firmware's reason if it fails."""
        if not self.query_bool(command):
            reason = self.last_error()
            raise OnStepError(f"{what} failed" + (f": {reason}" if reason else ""))

    def park(self) -> None:
        """Start moving to the park position. Tracking stops; the mount stays parked across power cycles."""
        self._run(":hP#", "park")

    def unpark(self) -> None:
        """Leave the park position. The firmware turns tracking back on."""
        self._run(":hR#", "unpark")

    def set_park_position(self) -> None:
        """Save the current position as the park position."""
        self._run(":hQ#", "setting the park position")

    def go_home(self) -> None:
        """Start moving to the home position (counterweight down, pointing at the pole)."""
        self._send(":hC#")  # replies nothing; ask for the result separately
        reason = self.last_error()
        if reason:
            raise OnStepError(f"going home failed: {reason}")

    def set_tracking(self, on: bool) -> None:
        self._run(":Te#" if on else ":Td#", "turning tracking " + ("on" if on else "off"))

    def stop(self) -> None:
        """Stop all motion at once: gotos, park, home and manual moves."""
        self._send(":Q#")

    def position(self) -> Position:
        """Where the mount points now (RA in hours, Dec in degrees)."""
        return Position(_sexagesimal(self.query(":GRa#")), _sexagesimal(self.query(":GDe#")))

    def goto(self, target: Position) -> None:
        """Slew to ``target`` at the current slew speed. The firmware checks horizon and axis limits."""
        if not self.query_bool(f":Sr{format_ra(target.ra_hours)}#"):
            raise OnStepError(f"the mount rejected target RA {format_ra(target.ra_hours)}")
        if not self.query_bool(f":Sd{format_dec(target.dec_degrees)}#"):
            raise OnStepError(f"the mount rejected target Dec {format_dec(target.dec_degrees)}")
        self._send(":MS#")
        reply = self._port.read(1)  # a single digit, no '#'
        if reply == b"0":
            return
        if reply and reply.isdigit():
            raise OnStepError("goto refused: " + GOTO_ERRORS.get(int(reply), f"error {reply.decode()}"))
        raise OnStepError(f"unexpected reply to :MS#: {reply!r}")

    def move(self, direction: str, degrees: float) -> Position:
        """Slew ``degrees`` north, south, east or west from where the mount points now. Returns the target."""
        target = self.position().moved(direction, degrees)
        self.goto(target)
        return target

    # --- site, time and other settings (command formats checked against OnStepX 10.23a) ----------------

    def site(self, now: datetime | None = None) -> Site:
        """Location, time zone and clock as the mount has them; ``now`` (UTC) is only for comparing clocks."""
        latitude = _sexagesimal(self.query(":GtH#"))
        longitude_west = _sexagesimal(self.query(":GgH#"))  # the mount stores east as negative
        to_utc = _sexagesimal(self.query(":GG#"))  # hours to ADD to local time to get UTC
        local_date = self.query(":GC#")
        local_time = self.query(":GL#")
        ready = not self.query_bool(":GX89#")  # inverted: a bare 0 means date and time are set
        clock_error = None
        try:
            month, day, year = (int(part) for part in local_date.split("/"))
            hour, minute, second = (int(float(part)) for part in local_time.split(":"))
            mount_utc = datetime(2000 + year, month, day, hour, minute, second, tzinfo=timezone.utc) \
                + timedelta(hours=to_utc)
            clock_error = (mount_utc - (now or datetime.now(timezone.utc))).total_seconds()
        except ValueError:
            pass
        return Site(latitude, -longitude_west, -to_utc, local_date, local_time, ready, clock_error)

    def set_location(self, latitude: float, longitude: float) -> None:
        """Latitude north positive, longitude EAST positive, both in decimal degrees. Saved in the mount."""
        if not -90 <= latitude <= 90:
            raise ValueError("latitude must be between -90 and 90")
        if not -180 <= longitude <= 180:
            raise ValueError("longitude must be between -180 and 180")
        self._run(f":St{_format_signed_dms(latitude, 2)}#", "setting the latitude")
        self._run(f":Sg{_format_signed_dms(-longitude, 3)}#", "setting the longitude")

    def sync_time(self, now: datetime | None = None) -> None:
        """Set the mount's time zone, date and time from this computer's clock (or ``now``)."""
        local = now if now is not None else datetime.now()
        if local.tzinfo is None:
            local = local.astimezone()  # this computer's time zone
        offset_hours = local.utcoffset().total_seconds() / 3600
        self._run(f":SG{format_utc_offset_for_mount(offset_hours)}#", "setting the time zone")
        self._run(f":SC{local:%m/%d/%y}#", "setting the date")
        if now is None:
            local = datetime.now().astimezone()  # don't lose the time spent on the commands above
        self._run(f":SL{local:%H:%M:%S}#", "setting the time")

    def limits(self) -> Limits:
        east = self._query_optional(":GXE9#")
        west = self._query_optional(":GXEA#")
        return Limits(
            horizon_deg=round(_sexagesimal(self.query(":Gh#"))),
            overhead_deg=round(_sexagesimal(self.query(":Go#"))),
            meridian_east_min=int(east) if east is not None else None,
            meridian_west_min=int(west) if west is not None else None,
        )

    def set_limits(self, horizon: int | None = None, overhead: int | None = None,
                   meridian_east: int | None = None, meridian_west: int | None = None) -> None:
        """Change any of the limits; the others stay as they are. Saved in the mount."""
        checks = [(horizon, -30, 30, "horizon limit"), (overhead, 60, 90, "overhead limit"),
                  (meridian_east, -1440, 1440, "meridian limit east"),
                  (meridian_west, -1440, 1440, "meridian limit west")]
        for value, low, high, name in checks:
            if value is not None and not (low <= value <= high and float(value).is_integer()):
                raise ValueError(f"{name} must be a whole number from {low} to {high}")
        if horizon is not None:
            self._run(f":Sh{int(horizon):+d}#", "setting the horizon limit")
        if overhead is not None:
            self._run(f":So{int(overhead):02d}#", "setting the overhead limit")
        if meridian_east is not None:
            self._run(f":SXE9,{int(meridian_east)}#", "setting the meridian limit east")
        if meridian_west is not None:
            self._run(f":SXEA,{int(meridian_west)}#", "setting the meridian limit west")

    def set_auto_meridian_flip(self, on: bool) -> None:
        self._run(f":SX95,{int(on)}#", "changing automatic meridian flip")

    def set_preferred_pier_side(self, side: str) -> None:
        """'E' (east), 'W' (west) or 'B' (best: stay on the current side when possible)."""
        if side not in ("E", "W", "B"):
            raise ValueError("pier side must be E, W or B")
        self._run(f":SX96,{side}#", "setting the preferred pier side")

    def set_refraction(self, on: bool) -> None:
        """Track with atmospheric refraction compensation (on) or at the plain rate (off)."""
        self._run(":Tr#" if on else ":Tn#", "changing refraction compensation")

    def set_buzzer(self, on: bool) -> None:
        self._run(f":SX97,{int(on)}#", "changing the buzzer")

    def settings(self) -> dict:
        """Everything the Settings section shows, read fresh from the mount."""
        status = self.status()
        auto_flip = self._query_optional(":GX95#")
        pier_side = self._query_optional(":GX96#")
        return {
            "site": self.site().to_dict(),
            "limits": self.limits().to_dict(),
            "auto_meridian_flip": auto_flip == "1" if auto_flip is not None else None,
            "preferred_pier_side": pier_side,
            "refraction": status.refraction,
            "buzzer": status.buzzer,
        }

    def set_slew_percent(self, percent: float) -> SlewSettings:
        """Set the slew speed as a percentage of the firmware's base speed (50-200)."""
        if not MIN_FACTOR * 100 <= percent <= MAX_FACTOR * 100:
            raise ValueError(f"percent must be between {MIN_FACTOR * 100:.0f} and {MAX_FACTOR * 100:.0f}")
        base = float(self.query(":GX93#"))
        return self.set_slew_us_per_step(base / (percent / 100))
