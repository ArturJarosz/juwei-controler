"""Find an OnStep mount among the USB serial ports."""

from __future__ import annotations

import errno
import os
from dataclasses import dataclass

import serial
from serial.tools import list_ports

from .onstep import DEFAULT_BAUD, DEFAULT_TIMEOUT, PRODUCT_PREFIX, OnStep, OnStepError

FALLBACK_BAUDS = (DEFAULT_BAUD, 115200)


@dataclass(frozen=True)
class ProbeResult:
    device: str
    description: str
    baud: int | None = None
    product: str | None = None
    version: str | None = None
    error: str | None = None
    # Why it failed: "permission", "permission_restart", "busy", "no_reply", "not_onstep" or "other".
    error_kind: str | None = None

    @property
    def is_mount(self) -> bool:
        return self.error is None and self.product is not None


def usb_serial_ports() -> list[list_ports.ListPortInfo]:
    """Serial ports backed by a USB device; built-in UARTs like /dev/ttyS0 are skipped."""
    return sorted((p for p in list_ports.comports() if p.vid is not None), key=lambda p: p.device)


def describe(port: list_ports.ListPortInfo) -> str:
    name = port.description or "USB serial"
    # Linux reports "<product> - <interface>", which for many adapters is the same text twice.
    head, separator, tail = name.partition(" - ")
    if separator and head == tail:
        name = head
    parts = [name]
    if port.vid is not None:
        parts.append(f"{port.vid:04x}:{port.pid:04x}")
    return ", ".join(parts)


def port_group(device: str) -> tuple[str, bool]:
    """The group that owns ``device`` (usually 'dialout'), and whether the user was added to it
    but this process started before that, so it doesn't have the group yet."""
    try:
        import grp
        import pwd

        gid = os.stat(device).st_gid
        group = grp.getgrgid(gid)
        user = pwd.getpwuid(os.getuid()).pw_name
    except (ImportError, OSError, KeyError):
        return "dialout", False
    return group.gr_name, user in group.gr_mem and gid not in os.getgroups()


def _open_failure(device: str, description: str, exc: Exception) -> ProbeResult:
    code = getattr(exc, "errno", None)
    if code == errno.EACCES:
        group, pending = port_group(device)
        if pending:
            return ProbeResult(device, description, error_kind="permission_restart",
                               error=f"permission denied: you are in the '{group}' group now, but this app was "
                                     f"started before that took effect (restart the computer, then start the app again)")
        return ProbeResult(device, description, error_kind="permission",
                           error=f"permission denied (add yourself to the '{group}' group, then restart the computer)")
    if code in (errno.EBUSY, errno.EAGAIN, errno.EWOULDBLOCK):
        return ProbeResult(device, description, error_kind="busy",
                           error="port is busy (another program is using it: KStars/INDI, NINA, or another copy of this app)")
    return ProbeResult(device, description, error=str(exc), error_kind="other")


def connect(device: str, description: str = "", bauds: tuple[int, ...] = FALLBACK_BAUDS,
            timeout: float = DEFAULT_TIMEOUT) -> tuple[OnStep | None, ProbeResult]:
    """Open ``device`` and check that an OnStep controller answers. On success the connection is left open."""
    last_error = "no reply"
    for baud in bauds:
        try:
            mount = OnStep.open(device, baud=baud, timeout=timeout)
        except (serial.SerialException, OSError) as exc:
            # Opening failed; another baud rate won't help.
            return None, _open_failure(device, description, exc)
        try:
            product = mount.product()
            if not product.startswith(PRODUCT_PREFIX):
                mount.close()
                return None, ProbeResult(device, description, baud, product, error_kind="not_onstep",
                                         error=f"not an OnStep mount ({product!r})")
            return mount, ProbeResult(device, description, baud, product, mount.firmware_version())
        except OnStepError as exc:
            mount.close()
            last_error = str(exc)
        except (serial.SerialException, OSError) as exc:
            mount.close()
            return None, _open_failure(device, description, exc)
    return None, ProbeResult(device, description, error=last_error, error_kind="no_reply")


def probe(device: str, description: str = "", bauds: tuple[int, ...] = FALLBACK_BAUDS,
          timeout: float = DEFAULT_TIMEOUT) -> ProbeResult:
    """Check whether ``device`` is an OnStep mount, without keeping the port open."""
    mount, result = connect(device, description, bauds, timeout)
    if mount is not None:
        mount.close()
    return result


def scan(bauds: tuple[int, ...] = FALLBACK_BAUDS, timeout: float = DEFAULT_TIMEOUT) -> list[ProbeResult]:
    """Probe every USB serial port."""
    return [probe(p.device, describe(p), bauds, timeout) for p in usb_serial_ports()]


def find_mount(bauds: tuple[int, ...] = FALLBACK_BAUDS, timeout: float = DEFAULT_TIMEOUT
               ) -> tuple[OnStep | None, ProbeResult | None, list[ProbeResult]]:
    """Open the first USB serial port with an OnStep mount. Also returns what was tried."""
    tried = []
    for port in usb_serial_ports():
        mount, result = connect(port.device, describe(port), bauds, timeout)
        tried.append(result)
        if mount is not None:
            return mount, result, tried
    return None, None, tried
