"""Continuous camera coverage through the real console IPC and loopback devices."""

import argparse
import ctypes
import json
import queue
import struct
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path

from test_manual_sequence import HeldPoseProbe
from test_station import Gui, valid_count


ROOT = Path(__file__).resolve().parents[2]


def wait_for(predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.03)
    raise AssertionError("condition timed out")


def pan_error(a, b):
    difference = abs(a - b) % 360
    return min(difference, 360 - difference)


class ContinuousPhotoProbe(HeldPoseProbe):
    """Record EF03/EF04 ordering and the physical pose at every photo request."""

    def __init__(self):
        self.photo_events = []
        super().__init__()

    def _fcs_request(self, raw, peer, now):
        if len(raw) >= 8:
            kind = struct.unpack_from("!H", raw)[0]
            if kind == 0xEF03 and len(raw) == 12:
                elevation, azimuth, _ = struct.unpack_from("!hhH", raw, 6)
                self.photo_events.append(("point", elevation / 10, azimuth / 10, now))
            elif kind == 0xEF04:
                enabled = struct.unpack_from("!H", raw, 6)[0]
                self.photo_events.append(("on" if enabled else "off",
                    self.elevation, self.azimuth,
                    self.target_elevation, self.target_azimuth, now))
        super()._fcs_request(raw, peer, now)


def run(executable):
    board = ContinuousPhotoProbe()
    # Deliberately register the right-hand object first. The wire commands must
    # preserve that registration order instead of sorting numeric IDs.
    first, second = 2002, 1001
    board.configure(targets=[
        {"id": first, "xyz": (.5, 5., .5)},
        {"id": second, "xyz": (-.5, 5., .5)},
    ])
    process = gui = None
    try:
        with tempfile.TemporaryDirectory(prefix="continuous-photo-",
                                         ignore_cleanup_errors=True) as directory:
            config = json.loads((ROOT / "configs/team-demo.json").read_text(encoding="utf-8"))
            for name, port in (("rdr", board.rdr_port), ("fcs", board.fcs_port)):
                config[name]["local"] = {"ip": "127.0.0.1", "port": 0}
                config[name]["peer"] = {"ip": "127.0.0.1", "port": port}
            config["camera"] = {"continuous_photo": [.05, 1, 0]}
            config_path = Path(directory) / "config.json"
            config_path.write_text(json.dumps(config), encoding="utf-8")
            event_log = Path(directory) / "events.log"
            suffix = "_continuous_photo_" + uuid.uuid4().hex
            process = subprocess.Popen([
                str(executable), "--config", str(config_path),
                "--ipc-suffix", suffix, "--event-log", str(event_log),
            ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            output = queue.Queue()

            def read_output():
                for line in process.stdout:
                    try:
                        output.put(json.loads(line))
                    except json.JSONDecodeError:
                        pass

            threading.Thread(target=read_output, daemon=True).start()
            assert output.get(timeout=10)["ready"]

            def ask(command):
                process.stdin.write(command + "\n")
                process.stdin.flush()
                return output.get(timeout=12)

            try:
                wait_for(lambda: ask("rdr status")["connected"])
                wait_for(lambda: ask("fcs status")["connected"])
            except Exception as error:
                raise AssertionError(("device connection failed", board.error,
                    board.snapshot(), process.poll(),
                    event_log.read_text(errors="replace") if event_log.exists() else "")) from error
            gui = Gui(suffix)
            assert gui.command(6, target=1) == 0
            gui.wait(lambda frame: valid_count(frame) == 2, timeout=15)

            assert gui.command(1201, target=first) == 0
            assert gui.command(1201, target=second) == 0
            assert gui.command(1204, target=first) == 0

            def paused():
                assert gui.command(1200, target=first) == 0
                return gui.last_response[8] & 1

            wait_for(paused)
            assert board.lidar_inhibited
            with board.lock:
                board.events.clear()
                board.photo_events.clear()

            assert gui.command(1210) == 0
            assert gui.last_response[6:9] == (18, 1, 0), gui.last_response

            # Read-only status remains available while every mutating batch
            # command is rejected for the full continuous-photo ownership.
            assert gui.command(1200, target=first) == 0
            assert gui.command(1206, target=first) != 0
            assert gui.command(1211) == 0

            confirmations = []
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                assert gui.command(1211) == 0
                total, phase, confirmed = gui.last_response[6:9]
                assert total == 18 and 0 <= confirmed <= total
                confirmations.append(confirmed)
                with board.lock:
                    acknowledged_or_pending = sum(event[0] == "on" for event in board.photo_events)
                assert confirmed <= acknowledged_or_pending
                if phase == 2:
                    break
                assert phase == 1, gui.last_response
                time.sleep(.04)
            else:
                raise AssertionError(("continuous photo completion timed out", gui.last_response,
                                      event_log.read_text(errors="replace")))

            assert confirmations == sorted(confirmations)
            assert confirmations[-1] == 18
            with board.lock:
                events = list(board.photo_events)
            assert [event[0] for event in events] == [kind for _ in range(18)
                for kind in ("point", "on")] + ["off"], events

            points = [event for event in events if event[0] == "point"]
            photos = [event for event in events if event[0] == "on"]
            assert len(points) == len(photos) == 18
            for photo in photos:
                _, elevation, azimuth, target_elevation, target_azimuth, _ = photo
                assert abs(elevation - target_elevation) <= .11, photo
                assert pan_error(azimuth, target_azimuth) <= .11, photo

            # Every square-coverage position must restart at object 1, then
            # object 2. The two base angles are far enough apart to identify
            # their subsequent offset commands without relying on ECS internals.
            first_base, second_base = points[0][2], points[1][2]
            assert pan_error(first_base, second_base) > 5
            for position in range(9):
                assert pan_error(points[position * 2][2], first_base) < 1
                assert pan_error(points[position * 2 + 1][2], second_base) < 1
            assert events[-1][0] == "off"
            assert board.laser_until <= time.monotonic()

            # A second run exercises explicit operator stop and its terminal
            # EF04(0), without waiting for all 18 images again.
            def start_again():
                result = gui.command(1210)
                assert result in (0, 12), gui.last_response
                return result == 0

            wait_for(start_again)
            with board.lock:
                second_run_start = len(board.photo_events)

            def has_one_photo():
                assert gui.command(1211) == 0
                return gui.last_response[8] >= 1

            wait_for(has_one_photo)
            assert gui.command(1212) == 0
            assert gui.last_response[7] == 3, gui.last_response
            with board.lock:
                stopped_events = list(board.photo_events[second_run_start:])
            assert any(event[0] == "on" for event in stopped_events)
            assert stopped_events[-1][0] == "off", stopped_events
            assert board.lidar_inhibited
            assert board.error is None, board.error

            gui.close()
            gui = None
            process.stdin.write("quit\n")
            process.stdin.flush()
            process.wait(timeout=10)
            assert process.returncode == 0, process.stderr.read()
            print("PASS: continuous photos IPC, ordered coverage, FCS acknowledgements, stop/off; loopback only")
    finally:
        if gui:
            gui.close()
        if process and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        board.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--console", type=Path, required=True)
    args = parser.parse_args()
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:
        run(args.console.resolve())
    finally:
        ctypes.windll.winmm.timeEndPeriod(1)
