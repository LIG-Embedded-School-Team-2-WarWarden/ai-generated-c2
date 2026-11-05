"""Binary ICD console against independent loopback RDR and FCS boards."""
import argparse
import ctypes
import json
import mmap
from pathlib import Path
import queue
import socket
import struct
import subprocess
import tempfile
import threading
import time
import uuid
from rdr_board import Board


class Gui:
    """Independent frame-v3 / command-v3 layout using Windows named objects."""
    def __init__(self, suffix):
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        self.kernel.OpenMutexW.argtypes = [ctypes.c_uint32, ctypes.c_int, ctypes.c_wchar_p]
        self.kernel.OpenMutexW.restype = ctypes.c_void_p
        self.kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self.kernel.ReleaseMutex.argtypes = [ctypes.c_void_p]
        self.kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        self.mutex = self.kernel.OpenMutexW(0x100001, False,
            'Local\\ControlStationMock.SharedMemory.Mutex.v3' + suffix)
        assert self.mutex, ctypes.get_last_error()
        self.memory = mmap.mmap(-1, 492, tagname='Local\\ControlStationMock.SharedMemory.v3' + suffix,
                                access=mmap.ACCESS_READ)
        self.pipe = open('\\\\.\\pipe\\ECS_NamedPipe' + suffix, 'r+b', buffering=0)
        self.sequence = 0

    def command(self, kind, target=0, rdr=1, fcs=1, xyz=(0, 0, 0), version=3):
        self.sequence += 1
        raw = struct.pack('<7I3f', 0x50495045, version, kind, self.sequence, rdr, fcs, target, *xyz)
        # Split the request to exercise the server's partial-read handling.
        self.pipe.write(raw[:17])
        self.pipe.write(raw[17:])
        reply = b''
        while len(reply) < 40:
            part = self.pipe.read(40 - len(reply))
            assert part, 'pipe disconnected'
            reply += part
        fields = struct.unpack('<10I', reply)
        assert fields[:4] == (0x50495045, 3, self.sequence, kind), fields
        return fields[4]

    def frame(self):
        assert self.kernel.WaitForSingleObject(self.mutex, 2000) in (0, 0x80)
        try:
            raw = self.memory[:]
        finally:
            self.kernel.ReleaseMutex(self.mutex)
        magic, version, size, sequence, timestamp, payload, checksum = struct.unpack_from('<4IQ2I', raw)
        assert (magic, version, size, payload) == (0x314D4354, 3, 492, 460)
        calculated = 2166136261
        for value in raw[32:]:
            calculated = ((calculated ^ value) * 16777619) & 0xffffffff
        assert checksum == calculated
        return raw

    def wait(self, predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            raw = self.frame()
            if predicate(raw):
                return raw
            time.sleep(.03)
        raise AssertionError('GUI frame did not reach requested state')

    def close(self):
        self.pipe.close()
        self.memory.close()
        self.kernel.CloseHandle(self.mutex)


def target_records(raw):
    count = struct.unpack_from('<I', raw, 128)[0]
    assert count <= 10
    return [struct.unpack_from('<I6fIBBH', raw, 132 + 36 * index)
            for index in range(count)]


def valid_count(raw):
    return sum(target[9] != 0 for target in target_records(raw))


class FcsBoard:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.settimeout(.05)
        self.port = self.socket.getsockname()[1]
        self.done = False
        self.error = None
        self.tx = self.elevation = 0
        self.azimuth = 1800  # Wire angles use 0.1 degrees.
        self.point_result = 0
        self.laser = False
        self.wire = []
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.done:
                try:
                    data, peer = self.socket.recvfrom(512)
                except socket.timeout:
                    continue
                kind, sequence, reply_to = struct.unpack_from('!3H', data)
                assert kind in (0xEF01, 0xEF02, 0xEF03, 0xEF04, 0xEF05)
                assert len(data) == (12 if kind == 0xEF03 else 8)
                assert reply_to == 0
                self.wire.append(data)
                self.tx = (self.tx + 1) & 65535
                if kind == 0xEF03:
                    elevation, azimuth, padding = struct.unpack_from('!hhH', data, 6)
                    assert padding == 0
                    if not self.point_result:
                        self.elevation, self.azimuth = elevation, azimuth
                if kind == 0xEF04:
                    value = struct.unpack_from('!H', data, 6)[0]
                    assert value in (0, 1)
                    self.laser = bool(value)
                header = struct.pack('!3H', kind + 0xF00, self.tx, sequence)
                if kind == 0xEF02:
                    response = header + struct.pack('!Hhh', 1 | (4 if self.laser else 0), self.elevation, self.azimuth)
                elif kind == 0xEF04:
                    response = header + struct.pack('!3H', 0, int(self.laser), 0)
                elif kind == 0xEF03:
                    response = header + struct.pack('!H', self.point_result)
                else:
                    response = header + bytes(2)
                self.socket.sendto(response, peer)
        except BaseException as error:
            self.error = error

    def close(self):
        self.done = True
        self.thread.join(2)
        self.socket.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    rdr, fcs = Board(), FcsBoard()
    process = None
    gui = None
    try:
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads((root / 'configs/team-demo.json').read_text(encoding='utf-8'))
            # An old configuration must no longer disable GUI assignment.
            config['assignment_enabled'] = False
            # Existing deployments may still supply the retired ECS limits.
            config['fcs']['limits'] = {'elevation_min': 0, 'elevation_max': 60,
                                       'azimuth_min': 120, 'azimuth_max': 240}
            config['rdr']['peer']['port'] = rdr.port
            config['fcs']['peer']['port'] = fcs.port
            config_path = Path(directory) / 'config.json'
            config_path.write_text(json.dumps(config), encoding='utf-8')
            suffix = '_test_' + uuid.uuid4().hex
            process = subprocess.Popen([str(args.console.resolve()), '--config', str(config_path),
                '--ipc-suffix', suffix],
                cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            output = queue.Queue()

            def read():
                for line in process.stdout:
                    try:
                        output.put(json.loads(line))
                    except json.JSONDecodeError:
                        pass

            threading.Thread(target=read, daemon=True).start()
            ready = output.get(timeout=5)
            assert ready['ready'] and ready['rdr_protocol'] == 'edde24' and ready['fcs_protocol'] == 'effe'
            assert rdr.starts == 0

            def ask(command):
                process.stdin.write(command + '\n')
                process.stdin.flush()
                return output.get(timeout=5)

            def wait(command, predicate, timeout=8):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    value = ask(command)
                    if predicate(value):
                        return value
                    time.sleep(.05)
                raise AssertionError((command, value))

            # Startup must connect without a console/GUI connect command.
            connected = wait('rdr status', lambda value: value['connected'])
            assert connected['mode'] == 0 and not connected['reporting'] and rdr.starts == 0
            assert any(packet[:2] == b'\xed\x01' for packet in rdr.wire)
            ask('rdr report on')
            wait('rdr status', lambda value: value['reporting'])
            ask('rdr start')
            wait('rdr status', lambda value: value['mode'] == 2)
            targets = wait('rdr targets', lambda value: len(value) == 2)
            target = next(t for t in targets if t['id'] == 7)
            assert all(abs(target[k]-v)<1e-6 for k,v in zip(('raw_x','raw_y','raw_z'),
                (.2427533743,-.5889764178,2.1896256709)))
            assert (target['observation_tilt'], target['observation_pan'],
                target['detected'], target['track_state']) == (123, 1980, 1, 1)
            assert target['world_valid']
            for field in ('world_xyz', 'gui_xyz'):
                assert all(abs(a-b)<1e-6 for a,b in zip(target[field], (.469,2.229,-.109)))
            ask('rdr exclude 7')
            wait('rdr targets', lambda value: any(t['id'] == 7 and t['status'] == 3 for t in value))
            ask('rdr resume 7')
            wait('rdr targets', lambda value: any(t['id'] == 7 and t['status'] == 0 for t in value))
            gui = Gui(suffix)
            frame = gui.wait(lambda raw: valid_count(raw) == 2)
            assert abs(struct.unpack_from('<f', frame, 48)[0] - 153) < .001
            assert abs(struct.unpack_from('<f', frame, 52)[0] - 12.3) < .001
            target = next(t for t in target_records(frame) if t[0] == 7)
            assert all(abs(a-b)<1e-6 for a,b in zip(target[4:7], (.469,2.229,-.109)))
            # Current status pose changes independently of the sample pose.
            rdr.pan, rdr.tilt = 2700, 400
            frame = gui.wait(lambda raw: abs(struct.unpack_from('<f', raw, 48)[0] - 225) < .001
                and abs(struct.unpack_from('<f', raw, 52)[0] - 40) < .001)
            target = next(t for t in target_records(frame) if t[0] == 7)
            assert all(abs(a-b)<1e-6 for a,b in zip(target[4:7], (.469,2.229,-.109)))
            rdr.track_state = 2
            wait('rdr targets', lambda rows: all(t['track_state'] == 2 for t in rows))
            frame = gui.wait(lambda raw: len(target_records(raw)) == 2 and valid_count(raw) == 2)
            target = next(t for t in target_records(frame) if t[0] == 7)
            assert target[8:10] == (0, 1)
            assert all(abs(a-b)<1e-6 for a,b in zip(target[4:7], (.469,2.229,-.109)))
            rdr.track_state = 0
            frame = gui.wait(lambda raw: valid_count(raw) == 0)
            assert {t[0] for t in target_records(frame)} == {7, 19}
            assert all(t[4:7] == (0, 0, 0) for t in target_records(frame))
            rdr.track_state = 1
            gui.wait(lambda raw: valid_count(raw) == 2)
            assert gui.command(0, version=2) == 1  # Old GUI version cannot control this station.
            wait('fcs status', lambda value: value['confirmed'] and value['connected'])
            points = sum(packet[:2] == b'\xef\x03' for packet in fcs.wire)
            assert gui.command(1, target=19) == 0
            assert sum(packet[:2] == b'\xef\x03' for packet in fcs.wire) == points + 1
            assert fcs.elevation == 0 and fcs.azimuth == 1639
            assert gui.command(2, target=19) == 0
            assert gui.command(1, target=7) == 0
            assert (fcs.elevation, fcs.azimuth) == (-27, 1919)
            assert gui.command(2, target=7) == 0
            fcs.point_result = 2  # Device INVALID_VALUE must remain a device rejection.
            points = sum(packet[:2] == b'\xef\x03' for packet in fcs.wire)
            assert gui.command(1, target=7) == 6
            assert sum(packet[:2] == b'\xef\x03' for packet in fcs.wire) == points + 1
            fcs.point_result = 0
            # The same fixed (east,north,up)=(2,3,1) observed at two poses.
            # Raw Y at the second pose is negative, but corrected height is +1:
            # using raw axes for the FCS would produce a different command angle.
            original_observations = rdr.observations
            for xyz, tilt, pan, world, wire_angles in (
                ((-2, 1, 3), 0, 1800, (2, 3, 1), (155, 2137)),
                ((3, -.1339745962, 2.2320508076), 300, 2700, (2, 3, 1), (155, 2137)),
                # Reported coordinates with right-positive FCS PAN.
                ((-1.044138, -.292257, 1.839167), 0, 1800,
                 (1.044138, 1.839167, -.292257), (-79, 2096)),
            ):
                rdr.track_state = 0
                gui.wait(lambda raw: valid_count(raw) == 0)
                rdr.observations = [original_observations[0], (19, xyz, tilt, pan)]
                rdr.track_state = 1
                wait('rdr targets', lambda rows: any(t['id'] == 19 and
                    t['observation_tilt'] == tilt and t['observation_pan'] == pan for t in rows))
                frame = gui.wait(lambda raw: any(t[0] == 19 and t[9] and
                    all(abs(a-b)<1e-5 for a,b in zip(t[4:7], world)) for t in target_records(raw)))
                assert gui.command(1, target=19) == 0
                assert (fcs.elevation, fcs.azimuth) == wire_angles
                assert gui.command(2, target=19) == 0
                # Display the latest measurement for the same ID; the held
                # position used by existing commands remains independent.
                rdr.observations = [original_observations[0], (19, (-4,2,5), 0, 1800)]
                wait('rdr targets', lambda rows: any(t['id']==19 and
                    t['gui_xyz']==[4,5,2] for t in rows))
                frame = gui.wait(lambda raw: any(t[0]==19 and t[9] and
                    all(abs(a-b)<1e-5 for a,b in zip(t[4:7],(4,5,2))) for t in target_records(raw)))
                assert gui.command(1,target=19)==0
                assert (fcs.elevation,fcs.azimuth)==wire_angles
                assert gui.command(2,target=19)==0
            rdr.track_state = 0
            gui.wait(lambda raw: valid_count(raw) == 0)
            rdr.observations = original_observations
            rdr.track_state = 1
            wait('rdr targets', lambda rows: any(t['id'] == 19 and t['observation_pan'] == 1980 for t in rows))
            assert gui.command(5, rdr=1, fcs=0, xyz=(1.25, -2.5, .75)) == 0
            assert gui.command(5, rdr=0, fcs=1, xyz=(-.5, 3, 1.5)) == 0
            frame = gui.wait(lambda raw: struct.unpack_from('<3f', raw, 56) == (1.25, -2.5, .75)
                and struct.unpack_from('<3f', raw, 100) == (-.5, 3, 1.5))
            target = next(t for t in target_records(frame) if t[0] == 7)
            assert target[0] == 7 and abs(target[4] - (1.25 + .469)) < .001
            assert abs(target[5] - (-2.5 + 2.229)) < .001 and abs(target[6] - (.75 - .109)) < .001
            assert gui.command(5, rdr=1, fcs=1) == 1
            assert gui.command(5, rdr=1, fcs=0, xyz=(float('nan'), 0, 0)) == 1
            assert gui.command(4, target=123) == 5  # RDR ED/DE has no elevation command.
            assert gui.command(4, target=901) == 1
            assert gui.command(6, target=2) == 1
            assert gui.command(6, target=0) == 0
            gui.wait(lambda raw: struct.unpack_from('<I', raw, 40)[0] == 0 and valid_count(raw) == 0)
            points = sum(packet[:2] == b'\xef\x03' for packet in fcs.wire)
            assert gui.command(1, target=19) == 2  # Stopped RDR still prevents assignment.
            assert sum(packet[:2] == b'\xef\x03' for packet in fcs.wire) == points
            wait('rdr status', lambda value: value['mode'] == 0)
            ask('rdr report off')
            wait('rdr status', lambda value: not value['reporting'])
            assert gui.command(6, target=1) == 0
            gui.wait(lambda raw: struct.unpack_from('<I', raw, 40)[0] == 1 and valid_count(raw) == 2)
            # GUI positions survive subsequent publication and stop/start cycles.
            assert struct.unpack_from('<3f', gui.frame(), 56) == (1.25, -2.5, .75)
            wait('fcs status', lambda value: value['confirmed'] and value['connected'])
            assert ask('fcs point 20 180')['result'] == 0
            status = ask('fcs status')
            assert status['elevation'] == 20 and status['azimuth'] == 180
            assert fcs.elevation == 200 and fcs.azimuth == 1800
            assert ask('fcs point 20.5 180.2')['result'] == 0
            assert fcs.elevation == 205 and fcs.azimuth == 1802
            status = ask('fcs status')
            assert status['elevation'] == 20.5 and status['azimuth'] == 180.2
            gui.wait(lambda raw: abs(struct.unpack_from('<f', raw, 92)[0] - 135.2) < .001
                and abs(struct.unpack_from('<f', raw, 96)[0] - 20.5) < .001)
            for command, wire_angles in (('fcs point -7.9 150.4', (-79, 1504)),
                                         ('fcs point 99 400', (990, 4000))):
                assert ask(command)['result'] == 0
                assert (fcs.elevation, fcs.azimuth) == wire_angles
            points = sum(packet[:2] == b'\xef\x03' for packet in fcs.wire)
            for bad in ('fcs point 3276.8 180', 'fcs point 20 -3276.8', 'fcs point 20.55 180', 'fcs point nan 180', 'fcs point 20 inf', 'fcs point 20 180tail', 'fcs point 20 180 extra', 'rdr start extra'):
                assert 'rejected' in ask(bad)
            assert points == sum(packet[:2] == b'\xef\x03' for packet in fcs.wire)
            assert ask('fcs laser on')['result'] == 0
            assert fcs.laser
            wait('fcs status', lambda value: not value['flags'] & 4)
            assert not fcs.laser
            ask('rdr stop')
            wait('rdr status', lambda value: value['mode'] == 0)
            process.stdin.write('quit\n')
            process.stdin.flush()
            process.wait(timeout=8)
            assert process.returncode == 0, process.stderr.read()
            assert rdr.error is None and fcs.error is None, (rdr.error, fcs.error)
            assert all(len(packet) == 24 for packet in rdr.wire)
            assert all(len(packet) in (8, 12) for packet in fcs.wire)
            print('PASS: RDR/FCS UDP + GUI v3, world coordinates, right-positive FCS PAN, negative tilt, retired limits ignored, device rejection propagated, wire validation, XYZ editing and shutdown')
    finally:
        if gui:
            gui.close()
        if process and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        rdr.close()
        fcs.close()


if __name__ == '__main__':
    main()
