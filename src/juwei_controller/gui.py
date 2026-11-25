"""Browser UI: a small web server on this computer that shows the mount and changes its slew speed.

It listens on 127.0.0.1 only, so other machines on the network cannot reach it.
"""

from __future__ import annotations

import errno
import json
import threading
from dataclasses import asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources

import serial

from .discovery import FALLBACK_BAUDS, ProbeResult, connect, find_mount
from .onstep import DEFAULT_TIMEOUT, OnStep, OnStepError

DEFAULT_HTTP_PORT = 8017


class NotConnected(Exception):
    pass


class MountService:
    """Keeps one connection to the mount open between requests and reconnects when it is plugged back in."""

    def __init__(self, port: str | None = None, bauds: tuple[int, ...] = FALLBACK_BAUDS,
                 timeout: float = DEFAULT_TIMEOUT):
        self._port = port
        self._bauds = bauds
        self._timeout = timeout
        self._lock = threading.Lock()
        self._mount: OnStep | None = None
        self._info: ProbeResult | None = None
        self._firmware_date: str | None = None
        self._tried: list[ProbeResult] = []
        self._lost: str | None = None
        self.paused = False

    def _ensure_connected(self) -> bool:
        if self._mount is not None:
            return True
        if self.paused:
            return False
        if self._port:
            mount, result = connect(self._port, self._port, self._bauds, self._timeout)
            info, tried = (result if mount else None), [result]
        else:
            mount, info, tried = find_mount(self._bauds, self._timeout)
        self._mount, self._info, self._tried = mount, info, tried
        if mount is None:
            return False
        self._lost = None
        try:
            self._firmware_date = mount.firmware_date()
        except OnStepError:
            self._firmware_date = None
        return True

    def _drop(self, reason: str | None = None) -> None:
        if self._mount is not None:
            self._mount.close()
        self._mount = None
        self._lost = reason

    def state(self) -> dict:
        with self._lock:
            if not self._ensure_connected():
                return self._offline()
            try:
                status = self._mount.status()
                slew = self._mount.slew_settings()
            except (OnStepError, serial.SerialException, OSError) as exc:
                self._drop(str(exc))
                return self._offline()
            try:
                position = self._mount.position().to_dict()
            except OnStepError:
                position = None
            return {
                "connected": True,
                "device": self._info.device,
                "description": self._info.description,
                "baud": self._info.baud,
                "product": self._info.product,
                "version": self._info.version,
                "firmware_date": self._firmware_date,
                "status": status.to_dict(),
                "position": position,
                "slew": slew.to_dict(),
            }

    def _offline(self) -> dict:
        return {
            "connected": False,
            "paused": self.paused,
            "lost": self._lost,
            "ports": [asdict(r) for r in self._tried],
        }

    def set_slew(self, request: dict) -> dict:
        with self._lock:
            if not self._ensure_connected():
                raise NotConnected("the mount is not connected")
            try:
                if "preset" in request:
                    after = self._mount.set_slew_preset(int(request["preset"]))
                elif "percent" in request:
                    after = self._mount.set_slew_percent(float(request["percent"]))
                elif "us" in request:
                    after = self._mount.set_slew_us_per_step(float(request["us"]))
                else:
                    raise ValueError("send one of: preset, percent, us")
            except (serial.SerialException, OSError) as exc:
                self._drop(str(exc))
                raise NotConnected(f"lost connection to the mount ({exc})") from exc
            return after.to_dict()

    def control(self, action: str) -> None:
        """Run a mount action: park, unpark, set_park, home, tracking_on, tracking_off or stop."""
        actions = {
            "park": OnStep.park,
            "unpark": OnStep.unpark,
            "set_park": OnStep.set_park_position,
            "home": OnStep.go_home,
            "tracking_on": lambda mount: mount.set_tracking(True),
            "tracking_off": lambda mount: mount.set_tracking(False),
            "stop": OnStep.stop,
        }
        if action not in actions:
            raise ValueError(f"unknown action {action!r}; use one of: {', '.join(actions)}")
        with self._lock:
            if not self._ensure_connected():
                raise NotConnected("the mount is not connected")
            try:
                actions[action](self._mount)
            except (serial.SerialException, OSError) as exc:
                self._drop(str(exc))
                raise NotConnected(f"lost connection to the mount ({exc})") from exc

    def move(self, direction: str, degrees: float) -> dict:
        """Slew ``degrees`` in ``direction`` from the current position; returns the target."""
        with self._lock:
            if not self._ensure_connected():
                raise NotConnected("the mount is not connected")
            try:
                return self._mount.move(direction, degrees).to_dict()
            except (serial.SerialException, OSError) as exc:
                self._drop(str(exc))
                raise NotConnected(f"lost connection to the mount ({exc})") from exc

    def _with_mount(self, work):
        with self._lock:
            if not self._ensure_connected():
                raise NotConnected("the mount is not connected")
            try:
                return work(self._mount)
            except (serial.SerialException, OSError) as exc:
                self._drop(str(exc))
                raise NotConnected(f"lost connection to the mount ({exc})") from exc

    def settings(self) -> dict:
        return self._with_mount(lambda mount: mount.settings())

    def change_setting(self, body: dict) -> dict:
        """Apply one settings change from the page, then return all settings read back from the mount."""
        name = body.get("setting")

        def number(key: str) -> float | None:
            value = body.get(key)
            return None if value is None or value == "" else float(value)

        def apply(mount: OnStep) -> dict:
            if name == "location":
                latitude, longitude = number("latitude"), number("longitude")
                if latitude is None or longitude is None:
                    raise ValueError("enter both latitude and longitude")
                mount.set_location(latitude, longitude)
            elif name == "time":
                mount.sync_time()
            elif name == "limits":
                mount.set_limits(number("horizon"), number("overhead"),
                                 number("meridian_east"), number("meridian_west"))
            elif name == "auto_meridian_flip":
                mount.set_auto_meridian_flip(bool(body.get("on")))
            elif name == "pier_side":
                mount.set_preferred_pier_side(str(body.get("side", "")))
            elif name == "refraction":
                mount.set_refraction(bool(body.get("on")))
            elif name == "buzzer":
                mount.set_buzzer(bool(body.get("on")))
            else:
                raise ValueError(f"unknown setting {name!r}")
            return mount.settings()

        return self._with_mount(apply)

    def disconnect(self) -> None:
        """Release the serial port so other software on this computer can use it."""
        with self._lock:
            self._drop()
            self._tried = []
            self.paused = True

    def resume(self) -> None:
        with self._lock:
            self.paused = False

    def close(self) -> None:
        with self._lock:
            self._drop()


class _Handler(BaseHTTPRequestHandler):
    server: _Server

    def log_message(self, format: str, *args) -> None:
        pass

    def _host_allowed(self) -> bool:
        # Rejecting other Host names blocks DNS-rebinding pages from driving the mount.
        port = self.server.server_address[1]
        return self.headers.get("Host") in (f"127.0.0.1:{port}", f"localhost:{port}")

    def _send(self, status: HTTPStatus, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: HTTPStatus, data: dict) -> None:
        self._send(status, json.dumps(data).encode(), "application/json")

    def do_GET(self) -> None:
        if not self._host_allowed():
            return self._json(HTTPStatus.FORBIDDEN, {"error": "unexpected Host header"})
        if self.path in ("/", "/index.html"):
            return self._send(HTTPStatus.OK, self.server.page, "text/html; charset=utf-8")
        if self.path == "/api/state":
            return self._json(HTTPStatus.OK, self.server.service.state())
        if self.path == "/api/settings":
            try:
                return self._json(HTTPStatus.OK, self.server.service.settings())
            except NotConnected as exc:
                return self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})
            except OnStepError as exc:
                return self._json(HTTPStatus.CONFLICT, {"error": str(exc)})
        self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:
        if not self._host_allowed():
            return self._json(HTTPStatus.FORBIDDEN, {"error": "unexpected Host header"})
        # Requiring JSON forces a CORS preflight, which this server never approves, so other web pages can't post here.
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            return self._json(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, {"error": "send application/json"})
        try:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(body, dict):
                raise ValueError("expected a JSON object")
        except ValueError as exc:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": f"bad request: {exc}"})

        service = self.server.service
        try:
            if self.path == "/api/slew":
                return self._json(HTTPStatus.OK, {"slew": service.set_slew(body)})
            if self.path == "/api/control":
                service.control(str(body.get("action", "")))
                return self._json(HTTPStatus.OK, service.state())
            if self.path == "/api/settings":
                return self._json(HTTPStatus.OK, service.change_setting(body))
            if self.path == "/api/move":
                target = service.move(str(body.get("direction", "")), float(body.get("degrees", 0)))
                return self._json(HTTPStatus.OK, {"target": target})
            if self.path == "/api/disconnect":
                service.disconnect()
                return self._json(HTTPStatus.OK, service.state())
            if self.path == "/api/connect":
                service.resume()
                return self._json(HTTPStatus.OK, service.state())
        except NotConnected as exc:
            return self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})
        except (ValueError, TypeError) as exc:
            return self._json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
        except OnStepError as exc:
            return self._json(HTTPStatus.CONFLICT, {"error": str(exc)})
        self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})


class _Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], service: MountService):
        super().__init__(address, _Handler)
        self.service = service
        self.page = resources.files(__package__).joinpath("gui.html").read_bytes()


def make_server(service: MountService, http_port: int = DEFAULT_HTTP_PORT) -> _Server:
    """Create the UI server on 127.0.0.1. Pass http_port=0 to pick a free port."""
    return _Server(("127.0.0.1", http_port), service)


def make_server_near(service: MountService, first_port: int = DEFAULT_HTTP_PORT, attempts: int = 10) -> _Server:
    """Create the UI server on first_port, or the next free port after it (e.g. when another copy is running)."""
    for port in range(first_port, first_port + attempts):
        try:
            return make_server(service, port)
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE or port == first_port + attempts - 1:
                raise
    raise AssertionError("unreachable")
