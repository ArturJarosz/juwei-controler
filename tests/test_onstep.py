import contextlib
import grp
import io
import json
import os
import pwd
import time
import unittest
from unittest import mock

from juwei_controller import cli
from juwei_controller.discovery import port_group, probe
from datetime import datetime, timedelta, timezone

from juwei_controller.onstep import (OnStep, OnStepError, Position, format_dec, format_deg_per_s, format_ra,
                                     format_utc_offset_for_mount, parse_status)
from juwei_controller.simulator import FakeOnStep, PtyServer

TIMEOUT = 0.3


class ParseStatusTest(unittest.TestCase):
    def test_idle_tracking_unparked(self):
        status = parse_status("pHaET150")
        self.assertTrue(status.tracking)
        self.assertTrue(status.slewing)  # no 'N' means a goto is running
        self.assertEqual(status.park, "unparked")
        self.assertTrue(status.at_home)
        self.assertTrue(status.auto_meridian_flip)
        self.assertEqual(status.error_code, 0)

    def test_parked_not_tracking_with_error(self):
        status = parse_status("nNPEo157")
        self.assertFalse(status.tracking)
        self.assertFalse(status.slewing)
        self.assertEqual(status.park, "parked")
        self.assertEqual(status.error, "meridian limit exceeded")

    def test_error_codes_above_nine_use_ascii_after_digits(self):
        self.assertEqual(parse_status("nNpE15?").error_code, 15)

    def test_unexpected_format_has_unknown_error(self):
        self.assertIsNone(parse_status("nNp").error_code)


class MountTestCase(unittest.TestCase):
    fake_kwargs: dict = {}

    def setUp(self):
        self.fake = FakeOnStep(**self.fake_kwargs)
        self.server = PtyServer(self.fake).start()
        self.addCleanup(self.server.stop)

    def open(self) -> OnStep:
        mount = OnStep.open(self.server.path, timeout=TIMEOUT)
        self.addCleanup(mount.close)
        return mount

    def run_cli(self, *argv: str) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            code = cli.main([*argv, "--port", self.server.path, "--timeout", str(TIMEOUT)])
        return code, out.getvalue()


class OnStepClientTest(MountTestCase):
    def test_reads_identity_and_slew_settings(self):
        mount = self.open()
        self.assertEqual(mount.product(), "On-Step")
        self.assertEqual(mount.firmware_version(), "10.24c")
        slew = mount.slew_settings()
        self.assertAlmostEqual(slew.base_us_per_step, 78.125)
        self.assertAlmostEqual(slew.factor, 1.0)
        self.assertAlmostEqual(slew.current_deg_per_s, 1.0)
        self.assertAlmostEqual(slew.max_factor, 2.0)

    def test_speed_is_exact_when_firmware_reports_steps_per_degree(self):
        mount = self.open()
        slew = mount.set_slew_percent(120)
        self.assertTrue(slew.speed_is_exact)
        self.assertEqual(slew.steps_per_degree, 12800.0)
        self.assertAlmostEqual(slew.current_deg_per_s, 1.2, places=3)  # the firmware itself would say "1.2"
        self.assertAlmostEqual(slew.base_deg_per_s, 1.0, places=6)
        self.assertAlmostEqual(slew.deg_per_s_at(slew.base_us_per_step / 1.37), 1.37, places=6)

    def test_steps_per_degree_is_read_once_per_connection(self):
        mount = self.open()
        mount.slew_settings()
        mount.slew_settings()
        self.assertEqual(self.fake.commands.count(":GXE4#"), 1)

    def test_preset_one_doubles_speed(self):
        slew = self.open().set_slew_preset(1)
        self.assertAlmostEqual(slew.factor, 2.0, places=3)
        self.assertIn(":SX93,1#", self.fake.commands)

    def test_preset_five_halves_speed(self):
        self.assertAlmostEqual(self.open().set_slew_preset(5).factor, 0.5, places=3)

    def test_percent_sets_rate_relative_to_base(self):
        slew = self.open().set_slew_percent(150)
        self.assertAlmostEqual(slew.current_us_per_step, 78.125 / 1.5, places=2)

    def test_percent_outside_range_is_rejected_without_sending(self):
        with self.assertRaises(ValueError):
            self.open().set_slew_percent(300)
        self.assertFalse(any(c.startswith(":SX92") for c in self.fake.commands))

    def test_refuses_to_change_while_slewing(self):
        self.fake.slewing = True
        with self.assertRaisesRegex(OnStepError, "moving"):
            self.open().set_slew_preset(1)
        self.assertAlmostEqual(self.fake.current_us, 78.125)

    def test_numeric_reply_reports_refusal(self):
        mount = self.open()
        self.fake.slewing = True
        self.assertFalse(mount.query_bool(":SX92,60#"))


class HardwareLimitTest(MountTestCase):
    fake_kwargs = {"fastest_us_per_step": 50.0}

    def test_max_factor_reflects_hardware_limit(self):
        self.assertAlmostEqual(self.open().slew_settings().max_factor, 78.125 / 50.0)

    def test_requests_above_hardware_limit_are_clamped(self):
        self.assertAlmostEqual(self.open().set_slew_percent(200).current_us_per_step, 50.0)

    def test_preset_clamped_by_hardware_limit_is_not_an_error(self):
        self.assertAlmostEqual(self.open().set_slew_preset(1).current_us_per_step, 50.0)


class LegacyFirmwareTest(MountTestCase):
    fake_kwargs = {"legacy": True}

    def test_missing_optional_commands_are_tolerated(self):
        slew = self.open().slew_settings()
        self.assertIsNone(slew.current_deg_per_s)
        self.assertIsNone(slew.base_deg_per_s)
        self.assertIsNone(slew.fastest_us_per_step)
        self.assertFalse(slew.speed_is_exact)
        self.assertAlmostEqual(slew.factor, 1.0)


class MismatchedStepsPerDegreeTest(MountTestCase):
    # A firmware fork that answers :GXE4# with something other than what :GX97# is based on.
    fake_kwargs = {"reported_steps_per_degree": 3200.0}

    def test_falls_back_to_estimate_from_reported_speed(self):
        slew = self.open().slew_settings()
        self.assertFalse(slew.speed_is_exact)
        self.assertIsNone(slew.steps_per_degree)
        self.assertAlmostEqual(slew.current_deg_per_s, 1.0)
        self.assertAlmostEqual(slew.base_deg_per_s, 1.0)


class FormatDegTest(unittest.TestCase):
    def test_exact_values_keep_up_to_two_decimals(self):
        self.assertEqual(format_deg_per_s(1.0), "1.0")
        self.assertEqual(format_deg_per_s(1.5), "1.5")
        self.assertEqual(format_deg_per_s(2 / 3), "0.67")
        self.assertEqual(format_deg_per_s(1.234), "1.23")

    def test_estimates_are_marked(self):
        self.assertEqual(format_deg_per_s(1.234, exact=False), "~1.2")


class DiscoveryTest(MountTestCase):
    def test_probe_identifies_mount(self):
        result = probe(self.server.path, timeout=TIMEOUT)
        self.assertTrue(result.is_mount)
        self.assertEqual(result.baud, 9600)
        self.assertEqual(result.version, "10.24c")

    def test_probe_reports_missing_port(self):
        result = probe("/dev/does-not-exist", timeout=TIMEOUT)
        self.assertFalse(result.is_mount)
        self.assertIsNotNone(result.error)
        self.assertEqual(result.error_kind, "other")

    @unittest.skipIf(os.geteuid() == 0, "root can open any file")
    def test_probe_reports_permission_denied(self):
        mode = os.stat(self.server.path).st_mode
        os.chmod(self.server.path, 0)
        self.addCleanup(os.chmod, self.server.path, mode)
        result = probe(self.server.path, timeout=TIMEOUT)
        self.assertEqual(result.error_kind, "permission")
        owner = grp.getgrgid(os.stat(self.server.path).st_gid).gr_name  # 'dialout' for real ports, 'tty' here
        self.assertIn(f"'{owner}' group", result.error)

    @unittest.skipIf(os.geteuid() == 0, "root can open any file")
    def test_probe_tells_user_to_restart_when_group_was_added_but_not_active(self):
        mode = os.stat(self.server.path).st_mode
        os.chmod(self.server.path, 0)
        self.addCleanup(os.chmod, self.server.path, mode)
        with mock.patch("juwei_controller.discovery.port_group", return_value=("dialout", True)):
            result = probe(self.server.path, timeout=TIMEOUT)
        self.assertEqual(result.error_kind, "permission_restart")
        self.assertIn("restart the computer", result.error)

    def test_port_group_detects_membership_not_yet_active(self):
        gid = os.stat(self.server.path).st_gid
        me = pwd.getpwuid(os.getuid()).pw_name
        group = grp.struct_group(("dialout", "x", gid, [me]))
        with mock.patch("grp.getgrgid", return_value=group), \
             mock.patch("os.getgroups", return_value=[g for g in os.getgroups() if g != gid]):
            self.assertEqual(port_group(self.server.path), ("dialout", True))
        with mock.patch("grp.getgrgid", return_value=group), mock.patch("os.getgroups", return_value=[gid]):
            self.assertEqual(port_group(self.server.path), ("dialout", False))

    def test_probe_reports_silent_device(self):
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        result = probe(os.ttyname(slave), timeout=TIMEOUT)
        self.assertEqual(result.error_kind, "no_reply")


class MountControlTest(MountTestCase):
    fake_kwargs = {"motion_seconds": 0.2}

    def wait_until_still(self, mount: OnStep) -> None:
        for _ in range(50):
            if not mount.status().moving:
                return
            time.sleep(0.05)
        self.fail("the mount kept moving")

    def test_park_moves_then_parks_and_stops_tracking(self):
        mount = self.open()
        mount.park()
        status = mount.status()
        self.assertEqual(status.park, "parking")
        self.assertTrue(status.slewing)
        self.wait_until_still(mount)
        status = mount.status()
        self.assertEqual(status.park, "parked")
        self.assertFalse(status.tracking)

    def test_unpark_turns_tracking_on(self):
        mount = self.open()
        mount.park()
        self.wait_until_still(mount)
        mount.unpark()
        status = mount.status()
        self.assertEqual(status.park, "unparked")
        self.assertTrue(status.tracking)

    def test_park_without_park_position_explains_why(self):
        self.fake.park_saved = False
        with self.assertRaisesRegex(OnStepError, "park failed: no park position has been set"):
            self.open().park()

    def test_set_park_position_then_park_works(self):
        self.fake.park_saved = False
        mount = self.open()
        mount.set_park_position()
        mount.park()
        self.assertEqual(mount.status().park, "parking")

    def test_unpark_when_not_parked_explains_why(self):
        with self.assertRaisesRegex(OnStepError, "not parked"):
            self.open().unpark()

    def test_tracking_off_and_on(self):
        mount = self.open()
        mount.set_tracking(False)
        self.assertFalse(mount.status().tracking)
        mount.set_tracking(True)
        self.assertTrue(mount.status().tracking)

    def test_tracking_on_while_parked_explains_why(self):
        mount = self.open()
        mount.park()
        self.wait_until_still(mount)
        with self.assertRaisesRegex(OnStepError, "parked"):
            mount.set_tracking(True)

    def test_go_home_reaches_home(self):
        mount = self.open()
        mount.go_home()
        self.assertTrue(mount.status().homing)
        self.wait_until_still(mount)
        self.assertTrue(mount.status().at_home)

    def test_go_home_while_parked_explains_why(self):
        mount = self.open()
        mount.park()
        self.wait_until_still(mount)
        with self.assertRaisesRegex(OnStepError, "going home failed: the mount is parked"):
            mount.go_home()

    def test_stop_aborts_park(self):
        mount = self.open()
        mount.park()
        mount.stop()
        status = mount.status()
        self.assertFalse(status.slewing)
        self.assertEqual(status.park, "unparked")


class PositionTest(unittest.TestCase):
    def test_format_and_parse_round_trip(self):
        self.assertEqual(format_ra(5.5), "05:30:00")
        self.assertEqual(format_ra(23.9999999), "00:00:00")
        self.assertEqual(format_dec(-8.2014), "-08*12:05")
        self.assertEqual(format_dec(22.0), "+22*00:00")

    def test_north_and_south_change_dec(self):
        self.assertAlmostEqual(Position(5.5, 20).moved("north", 10).dec_degrees, 30)
        self.assertAlmostEqual(Position(5.5, 20).moved("south", 10).dec_degrees, 10)

    def test_east_and_west_turn_ra_and_wrap(self):
        self.assertAlmostEqual(Position(23.5, 0).moved("east", 15).ra_hours, 0.5)
        self.assertAlmostEqual(Position(0.25, 0).moved("west", 15).ra_hours, 23.25)

    def test_past_the_pole_is_refused(self):
        with self.assertRaisesRegex(ValueError, "past the pole"):
            Position(5.5, 85).moved("north", 10)

    def test_bad_input_is_refused(self):
        with self.assertRaises(ValueError):
            Position(5.5, 0).moved("up", 10)
        with self.assertRaises(ValueError):
            Position(5.5, 0).moved("north", 0)


class MoveTest(MountTestCase):
    fake_kwargs = {"motion_seconds": 0.2}

    def test_reads_position(self):
        position = self.open().position()
        self.assertAlmostEqual(position.ra_hours, 5.5)
        self.assertAlmostEqual(position.dec_degrees, 20.0)

    def test_move_north_slews_and_arrives(self):
        mount = self.open()
        target = mount.move("north", 10)
        self.assertAlmostEqual(target.dec_degrees, 30.0)
        self.assertTrue(mount.status().slewing)
        for _ in range(50):
            if not mount.status().moving:
                break
            time.sleep(0.05)
        position = mount.position()
        self.assertAlmostEqual(position.dec_degrees, 30.0, places=3)
        self.assertAlmostEqual(position.ra_hours, 5.5, places=3)
        self.assertIn(":Sd+30*00:00#", self.fake.commands)

    def test_move_east_sends_ra_forty_minutes_later(self):
        self.open().move("east", 10)
        self.assertIn(":Sr06:10:00#", self.fake.commands)

    def test_past_the_pole_sends_no_goto(self):
        self.fake.dec_degrees = 85
        with self.assertRaises(ValueError):
            self.open().move("north", 10)
        self.assertNotIn(":MS#", self.fake.commands)

    def test_parked_mount_explains_why(self):
        self.fake.park = "parked"
        with self.assertRaisesRegex(OnStepError, "goto refused: the mount is parked"):
            self.open().move("north", 10)

    def test_mount_without_time_explains_why(self):
        self.fake.time_ready = False
        with self.assertRaisesRegex(OnStepError, "date and time may not be set"):
            self.open().move("north", 10)

    def test_below_horizon_explains_why(self):
        self.fake.dec_degrees = -35
        with self.assertRaisesRegex(OnStepError, "below the horizon"):
            self.open().move("south", 10)


class SettingsFormatTest(unittest.TestCase):
    def test_utc_offset_is_sent_with_the_mounts_opposite_sign(self):
        self.assertEqual(format_utc_offset_for_mount(1), "-01:00")
        self.assertEqual(format_utc_offset_for_mount(5.5), "-05:30")
        self.assertEqual(format_utc_offset_for_mount(5.75), "-05:45")
        self.assertEqual(format_utc_offset_for_mount(-3), "+03:00")
        with self.assertRaises(ValueError):
            format_utc_offset_for_mount(5.2)

    def test_status_reports_refraction_and_buzzer(self):
        self.assertEqual(parse_status("NprszaE150").refraction, "refraction")
        self.assertTrue(parse_status("NprszaE150").buzzer)
        self.assertEqual(parse_status("NptsE150").refraction, "full model")
        self.assertEqual(parse_status("NpE150").refraction, "off")
        self.assertFalse(parse_status("NpE150").buzzer)


class SettingsTest(MountTestCase):
    def test_reads_site(self):
        site = self.open().site()
        self.assertAlmostEqual(site.latitude, 47.0)
        self.assertAlmostEqual(site.longitude, 8.5)  # east positive, although the mount stores -8.5
        self.assertEqual(site.utc_offset_hours, 1.0)
        self.assertTrue(site.date_time_ready)
        self.assertLess(abs(site.clock_error_s), 2)

    def test_set_location_east(self):
        mount = self.open()
        mount.set_location(47.3769, 8.5417)
        self.assertIn(":St+47*22:37#", self.fake.commands)
        self.assertIn(":Sg-008*32:30#", self.fake.commands)  # east is negative on the mount
        site = mount.site()
        self.assertAlmostEqual(site.latitude, 47.3769, places=3)
        self.assertAlmostEqual(site.longitude, 8.5417, places=3)

    def test_set_location_south_west(self):
        mount = self.open()
        mount.set_location(-33.8568, -73.9857)
        self.assertIn(":St-33*51:24#", self.fake.commands)
        self.assertIn(":Sg+073*59:09#", self.fake.commands)
        self.assertAlmostEqual(mount.site().longitude, -73.9857, places=3)

    def test_location_out_of_range_sends_nothing(self):
        with self.assertRaises(ValueError):
            self.open().set_location(95, 8)
        self.assertFalse(any(c.startswith((":St", ":Sg")) for c in self.fake.commands))

    def test_sync_time_sets_zone_date_and_time(self):
        self.fake.date_ready = self.fake.time_ready = False
        mount = self.open()
        self.assertFalse(mount.site().date_time_ready)
        india = timezone(timedelta(hours=5, minutes=30))
        now = datetime(2026, 11, 25, 22, 15, 0, tzinfo=india)
        mount.sync_time(now)
        self.assertIn(":SG-05:30#", self.fake.commands)
        self.assertIn(":SC11/25/26#", self.fake.commands)
        self.assertIn(":SL22:15:00#", self.fake.commands)
        site = mount.site(now=now.astimezone(timezone.utc))
        self.assertTrue(site.date_time_ready)
        self.assertEqual(site.utc_offset_hours, 5.5)
        self.assertLess(abs(site.clock_error_s), 2)

    def test_limits_read_and_write(self):
        mount = self.open()
        self.assertEqual(mount.limits().to_dict(), {"horizon_deg": -10, "overhead_deg": 85,
                                                    "meridian_east_min": 15, "meridian_west_min": 15})
        mount.set_limits(horizon=5, overhead=80, meridian_east=-10, meridian_west=30)
        for command in (":Sh+5#", ":So80#", ":SXE9,-10#", ":SXEA,30#"):
            self.assertIn(command, self.fake.commands)
        self.assertEqual(mount.limits().to_dict(), {"horizon_deg": 5, "overhead_deg": 80,
                                                    "meridian_east_min": -10, "meridian_west_min": 30})

    def test_only_given_limits_are_sent(self):
        self.open().set_limits(overhead=75)
        sent = [c for c in self.fake.commands if c.startswith((":Sh", ":So", ":SXE"))]
        self.assertEqual(sent, [":So75#"])

    def test_limits_out_of_range_send_nothing(self):
        mount = self.open()
        for kwargs in ({"horizon": 40}, {"overhead": 50}, {"meridian_east": 2000}, {"horizon": 2.5}):
            with self.assertRaises(ValueError, msg=kwargs):
                mount.set_limits(**kwargs)
        self.assertFalse(any(c.startswith((":Sh", ":So", ":SXE")) for c in self.fake.commands))

    def test_meridian_flip_settings(self):
        mount = self.open()
        mount.set_auto_meridian_flip(False)
        mount.set_preferred_pier_side("W")
        settings = mount.settings()
        self.assertFalse(settings["auto_meridian_flip"])
        self.assertEqual(settings["preferred_pier_side"], "W")
        self.assertFalse(mount.status().auto_meridian_flip)
        with self.assertRaises(ValueError):
            mount.set_preferred_pier_side("X")

    def test_refraction_and_buzzer(self):
        mount = self.open()
        mount.set_refraction(True)
        mount.set_buzzer(False)
        status = mount.status()
        self.assertEqual(status.refraction, "refraction")
        self.assertFalse(status.buzzer)
        mount.set_refraction(False)
        self.assertEqual(mount.status().refraction, "off")


class CliTest(MountTestCase):
    def test_status_shows_connection_and_slew(self):
        code, out = self.run_cli("status")
        self.assertEqual(code, 0, out)
        self.assertIn("connected on", out)
        self.assertIn("On-Step 10.24c", out)
        self.assertIn("1.0 deg/s, 100% of base", out)
        self.assertIn("base rate     78.125 us/step (100%, 1.0 deg/s)", out)

    def test_status_json(self):
        code, out = self.run_cli("status", "--json")
        self.assertEqual(code, 0, out)
        data = json.loads(out)
        self.assertTrue(data["connected"])
        self.assertEqual(data["slew"]["percent_of_base"], 100.0)
        self.assertEqual(data["status"]["park"], "unparked")

    def test_slew_preset_changes_and_reports(self):
        code, out = self.run_cli("slew", "--preset", "2")
        self.assertEqual(code, 0, out)
        self.assertIn("after   1.5 deg/s, 150% of base", out)
        self.assertAlmostEqual(self.fake.current_us, 78.125 / 1.5, places=2)

    def test_slew_without_options_only_reads(self):
        code, out = self.run_cli("slew")
        self.assertEqual(code, 0, out)
        self.assertIn("Slew speed  1.0 deg/s", out)
        self.assertFalse(any(c.startswith(":SX") for c in self.fake.commands))

    def test_slew_while_moving_fails_cleanly(self):
        self.fake.slewing = True
        code, out = self.run_cli("slew", "--percent", "150")
        self.assertEqual(code, cli.EXIT_ERROR)
        self.assertIn("moving", out)

    def test_tracking_off_command(self):
        code, out = self.run_cli("tracking", "off")
        self.assertEqual(code, 0, out)
        self.assertIn("Tracking off.", out)
        self.assertFalse(self.fake.tracking)

    def test_move_command(self):
        code, out = self.run_cli("move", "south", "5")
        self.assertEqual(code, 0, out)
        self.assertIn("Moving 5 deg south to RA 05:30:00  Dec +15 deg 00:00", out)

    def test_status_shows_pointing(self):
        code, out = self.run_cli("status")
        self.assertIn("Pointing    RA 05:30:00  Dec +20 deg 00:00", out)

    def test_settings_command(self):
        code, out = self.run_cli("settings")
        self.assertEqual(code, 0, out)
        self.assertIn("Location    latitude 47.0000, longitude 8.5000 (east positive)", out)
        self.assertIn("Limits      horizon -10 deg, overhead 85 deg", out)
        self.assertIn("matches this computer", out)

    def test_location_and_limits_commands(self):
        code, out = self.run_cli("location", "47.5", "8.25")
        self.assertEqual(code, 0, out)
        self.assertIn("Location saved.", out)
        self.assertIn("longitude 8.2500", out)
        code, out = self.run_cli("limits", "--horizon", "40")
        self.assertEqual(code, cli.EXIT_ERROR)
        self.assertIn("horizon limit must be a whole number from -30 to 30", out)

    def test_flip_and_buzzer_commands(self):
        code, out = self.run_cli("flip", "--auto", "off", "--side", "east")
        self.assertEqual(code, 0, out)
        self.assertIn("automatic flip off, preferred side east", out)
        code, out = self.run_cli("buzzer", "off")
        self.assertIn("Buzzer      off", out)

    def test_park_command(self):
        code, out = self.run_cli("park")
        self.assertEqual(code, 0, out)
        self.assertIn("Parking.", out)
        self.assertEqual(self.fake.park, "parking")

    def test_park_command_failure_is_reported(self):
        self.fake.park_saved = False
        code, out = self.run_cli("park")
        self.assertEqual(code, cli.EXIT_ERROR)
        self.assertIn("no park position has been set", out)

    def test_status_when_port_missing_is_not_connected(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = cli.main(["status", "--port", "/dev/does-not-exist", "--timeout", str(TIMEOUT)])
        self.assertEqual(code, cli.EXIT_NOT_CONNECTED)
        self.assertIn("not connected", out.getvalue())


if __name__ == "__main__":
    unittest.main()
