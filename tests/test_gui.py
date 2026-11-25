import http.client
import json
import socket
import threading
import unittest

from juwei_controller.gui import MountService, NotConnected, make_server, make_server_near
from juwei_controller.onstep import OnStepError
from juwei_controller.simulator import FakeOnStep, PtyServer

TIMEOUT = 0.3


class MountServiceTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeOnStep()
        self.pty = PtyServer(self.fake).start()
        self.addCleanup(self.pty.stop)
        self.service = MountService(self.pty.path, timeout=TIMEOUT)
        self.addCleanup(self.service.close)

    def test_state_when_connected(self):
        state = self.service.state()
        self.assertTrue(state["connected"])
        self.assertEqual(state["version"], "10.24c")
        self.assertEqual(state["firmware_date"], "Nov 25 2026")
        self.assertEqual(state["slew"]["percent_of_base"], 100.0)

    def test_keeps_connection_open_between_requests(self):
        self.service.state()
        self.service.state()
        self.assertEqual(self.fake.commands.count(":GVP#"), 1)

    def test_set_slew_by_preset_and_percent(self):
        self.assertEqual(self.service.set_slew({"preset": 2})["percent_of_base"], 150.0)
        self.assertEqual(self.service.set_slew({"percent": 80})["percent_of_base"], 80.0)

    def test_set_slew_rejects_unknown_request(self):
        with self.assertRaises(ValueError):
            self.service.set_slew({"speed": 3})

    def test_set_slew_while_moving_raises(self):
        self.fake.slewing = True
        with self.assertRaises(OnStepError):
            self.service.set_slew({"preset": 1})

    def test_release_and_reconnect(self):
        self.service.state()
        self.service.disconnect()
        state = self.service.state()
        self.assertFalse(state["connected"])
        self.assertTrue(state["paused"])
        with self.assertRaises(NotConnected):
            self.service.set_slew({"preset": 1})
        self.service.resume()
        self.assertTrue(self.service.state()["connected"])

    def test_missing_port_reports_not_connected(self):
        service = MountService("/dev/does-not-exist", timeout=TIMEOUT)
        state = service.state()
        self.assertFalse(state["connected"])
        self.assertEqual(state["ports"][0]["device"], "/dev/does-not-exist")
        self.assertTrue(state["ports"][0]["error"])


class PortFallbackTest(unittest.TestCase):
    def test_uses_next_free_port_when_first_is_taken(self):
        busy = socket.socket()
        self.addCleanup(busy.close)
        busy.bind(("127.0.0.1", 0))
        busy.listen()
        taken = busy.getsockname()[1]
        service = MountService("/dev/does-not-exist", timeout=TIMEOUT)

        server = make_server_near(service, first_port=taken)
        self.addCleanup(server.server_close)
        self.assertGreater(server.server_address[1], taken)
        self.assertLess(server.server_address[1], taken + 10)

    def test_explicit_port_that_is_taken_fails(self):
        busy = socket.socket()
        self.addCleanup(busy.close)
        busy.bind(("127.0.0.1", 0))
        busy.listen()
        with self.assertRaises(OSError):
            make_server(MountService("/dev/does-not-exist"), busy.getsockname()[1])


class HttpApiTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeOnStep()
        self.pty = PtyServer(self.fake).start()
        self.addCleanup(self.pty.stop)
        service = MountService(self.pty.path, timeout=TIMEOUT)
        self.server = make_server(service, http_port=0)
        self.port = self.server.server_address[1]
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()

        def shutdown():
            self.server.shutdown()
            self.server.server_close()
            service.close()
        self.addCleanup(shutdown)

    def request(self, method, path, body=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        self.addCleanup(conn.close)
        all_headers = {"Content-Type": "application/json"} if body is not None else {}
        all_headers.update(headers or {})
        conn.request(method, path, body=None if body is None else json.dumps(body), headers=all_headers)
        res = conn.getresponse()
        return res.status, res.getheader("Content-Type"), res.read()

    def test_serves_page(self):
        status, content_type, body = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", content_type)
        self.assertIn(b"Juwei-17 Controller", body)

    def test_state_endpoint(self):
        status, _, body = self.request("GET", "/api/state")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["connected"])

    def test_set_preset(self):
        status, _, body = self.request("POST", "/api/slew", {"preset": 1})
        self.assertEqual(status, 200, body)
        self.assertEqual(json.loads(body)["slew"]["percent_of_base"], 200.0)

    def test_invalid_percent_is_bad_request(self):
        status, _, body = self.request("POST", "/api/slew", {"percent": 999})
        self.assertEqual(status, 400)
        self.assertIn("between 50 and 200", json.loads(body)["error"])

    def test_moving_mount_is_conflict(self):
        self.fake.slewing = True
        status, _, _ = self.request("POST", "/api/slew", {"preset": 1})
        self.assertEqual(status, 409)

    def test_post_without_json_content_type_is_refused(self):
        status, _, _ = self.request("POST", "/api/slew", None, {"Content-Type": "text/plain", "Content-Length": "0"})
        self.assertEqual(status, 415)
        self.assertFalse(any(c.startswith(":SX") for c in self.fake.commands))

    def test_foreign_host_header_is_refused(self):
        status, _, _ = self.request("GET", "/api/state", headers={"Host": "evil.example:80"})
        self.assertEqual(status, 403)

    def test_control_park_returns_new_state(self):
        status, _, body = self.request("POST", "/api/control", {"action": "park"})
        self.assertEqual(status, 200, body)
        self.assertEqual(json.loads(body)["status"]["park"], "parking")

    def test_control_tracking_off(self):
        status, _, body = self.request("POST", "/api/control", {"action": "tracking_off"})
        self.assertEqual(status, 200, body)
        self.assertFalse(json.loads(body)["status"]["tracking"])

    def test_control_refused_by_mount_is_conflict_with_reason(self):
        status, _, body = self.request("POST", "/api/control", {"action": "unpark"})
        self.assertEqual(status, 409)
        self.assertIn("not parked", json.loads(body)["error"])

    def test_state_includes_position(self):
        _, _, body = self.request("GET", "/api/state")
        self.assertEqual(json.loads(body)["position"]["ra"], "05:30:00")

    def test_move_returns_target(self):
        status, _, body = self.request("POST", "/api/move", {"direction": "north", "degrees": 10})
        self.assertEqual(status, 200, body)
        self.assertEqual(json.loads(body)["target"]["dec"], "+30*00:00")

    def test_move_with_bad_degrees_is_bad_request(self):
        for degrees in ("ten", None, 500):
            status, _, _ = self.request("POST", "/api/move", {"direction": "north", "degrees": degrees})
            self.assertEqual(status, 400, degrees)
        self.assertNotIn(":MS#", self.fake.commands)

    def test_move_refused_by_mount_is_conflict(self):
        self.fake.park = "parked"
        status, _, body = self.request("POST", "/api/move", {"direction": "east", "degrees": 10})
        self.assertEqual(status, 409)
        self.assertIn("parked", json.loads(body)["error"])

    def test_get_settings(self):
        status, _, body = self.request("GET", "/api/settings")
        self.assertEqual(status, 200, body)
        settings = json.loads(body)
        self.assertEqual(settings["site"]["longitude"], 8.5)
        self.assertEqual(settings["limits"]["overhead_deg"], 85)
        self.assertEqual(settings["preferred_pier_side"], "B")

    def test_change_location_returns_new_settings(self):
        status, _, body = self.request("POST", "/api/settings",
                                       {"setting": "location", "latitude": 46.5, "longitude": 7.25})
        self.assertEqual(status, 200, body)
        self.assertAlmostEqual(json.loads(body)["site"]["latitude"], 46.5)

    def test_sync_time(self):
        self.fake.date_ready = self.fake.time_ready = False
        status, _, body = self.request("POST", "/api/settings", {"setting": "time"})
        self.assertEqual(status, 200, body)
        self.assertTrue(json.loads(body)["site"]["date_time_ready"])

    def test_toggles(self):
        _, _, body = self.request("POST", "/api/settings", {"setting": "refraction", "on": True})
        self.assertEqual(json.loads(body)["refraction"], "refraction")
        _, _, body = self.request("POST", "/api/settings", {"setting": "buzzer", "on": False})
        self.assertFalse(json.loads(body)["buzzer"])
        _, _, body = self.request("POST", "/api/settings", {"setting": "auto_meridian_flip", "on": False})
        self.assertFalse(json.loads(body)["auto_meridian_flip"])

    def test_bad_settings_are_bad_requests(self):
        for body in ({"setting": "colour"}, {"setting": "limits", "horizon": 45},
                     {"setting": "location", "latitude": 46.5}, {"setting": "pier_side", "side": "N"},
                     {"setting": "limits", "overhead": "high"}):
            status, _, _ = self.request("POST", "/api/settings", body)
            self.assertEqual(status, 400, body)

    def test_unknown_control_action_is_bad_request(self):
        status, _, body = self.request("POST", "/api/control", {"action": "dance"})
        self.assertEqual(status, 400)
        self.assertIn("unknown action", json.loads(body)["error"])

    def test_disconnect_and_connect(self):
        status, _, body = self.request("POST", "/api/disconnect", {})
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["paused"])
        status, _, body = self.request("POST", "/api/connect", {})
        self.assertTrue(json.loads(body)["connected"])


if __name__ == "__main__":
    unittest.main()
