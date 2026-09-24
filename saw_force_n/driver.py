"""Single-owner serial worker. Configuration/tare is never run in the GUI thread."""
import queue
import re
import threading
import time
from collections import deque
import serial
from .protocol import Framer, parse_sample


class Driver:
    def __init__(self, port, baud=115200, tare_timeout=60.0):
        self.port, self.baud, self.tare_timeout = port, baud, tare_timeout
        self.lock = threading.Lock()
        self.samples = deque(maxlen=2000)
        self.state = 'Connecting'
        self.force = None
        self.count = 0
        self.last = None
        self.periods = deque(maxlen=1000)
        self.tare_pending = False
        self.requests = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, name='force-n-serial', daemon=True)

    def snapshot(self):
        with self.lock:
            periods = list(self.periods)
            return dict(state=self.state, force=self.force, count=self.count,
                        age=None if self.last is None else time.monotonic()-self.last,
                        hz=1/(sum(periods)/len(periods)) if periods else 0,
                        minimum=min(periods, default=0), maximum=max(periods, default=0))

    def state_to(self, value):
        with self.lock:
            self.state = value
            if value != 'Streaming':
                self.force = None
                self.samples.clear()

    def request_tare(self):
        with self.lock:
            if self.state != 'Streaming' or self.tare_pending:
                return False
            self.tare_pending = True
            self.state = 'Tare requested'
            self.force = None
            self.samples.clear()
            self.requests.put_nowait('tare')
            return True

    def take_samples(self):
        with self.lock:
            result = list(self.samples) if self.state == 'Streaming' else []
            self.samples.clear()
            return result

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)

    def receive(self):
        return self.framer.feed(self.serial.read(min(4096, max(1, self.serial.in_waiting))))

    def command(self, packet, expected=b'e0x1', timeout=2):
        self.serial.write(packet)
        deadline = time.monotonic()+timeout
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            for frame in self.receive():
                if expected is None and re.fullmatch(rb'r0x[0-9a-fA-F]+', frame):
                    return frame[1:]
                if frame == expected:
                    return frame
                # e0x1 acknowledges tare start; e0x4 alone completes it.
                if frame.startswith(b'e') and frame != b'e0x1':
                    raise RuntimeError(f'{packet!r}: unexpected response {frame!r}')
        raise RuntimeError(f'{packet!r}: cancelled or response timeout')

    def configure(self):
        self.command(b'<SDM0>')
        uid = self.command(b'<GI1>', expected=None)
        self.command(b'<SEL'+uid+b'>')
        self.command(b'<TDO2>')
        self.command(b'<TOS0x10>')
        self.command(b'<SDM2>')

    def tare(self):
        self.state_to('Taring: keep sensor unloaded and stationary')
        self.command(b'<SDM0>')
        self.command(b'<SDM3>', expected=b'e0x4', timeout=self.tare_timeout)
        self.command(b'<SDM2>')
        with self.lock:
            self.tare_pending = False
            self.last = None
            self.periods.clear()
        self.state_to('Streaming')

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=.1, write_timeout=1, exclusive=True) as self.serial:
                self.framer = Framer()
                try:
                    self.configure()
                    self.state_to('Streaming')
                    deadline = time.monotonic()+2
                    while not self.stop_event.is_set():
                        if not self.requests.empty():
                            self.requests.get_nowait()
                            self.tare()
                            deadline = time.monotonic()+2
                        for frame in self.receive():
                            values = parse_sample(frame)
                            if values is None:
                                if frame != b'e0x1':
                                    raise RuntimeError(f'Unexpected sensor response: {frame!r}')
                                continue
                            if len(values) != 3:
                                raise RuntimeError(f'Expected three calibrated channels; received {len(values)}')
                            now = time.monotonic()
                            force = tuple(v*.001 for v in values)
                            with self.lock:
                                if self.last is not None:
                                    self.periods.append(now-self.last)
                                self.last = now
                                self.count += 1
                                if self.state == 'Streaming':
                                    self.force = force
                                    self.samples.append(force)
                            deadline = now+2
                        if time.monotonic() > deadline:
                            raise RuntimeError('No valid sensor samples for 2 seconds')
                finally:
                    self.serial.write(b'<SDM0>')
        except Exception as exc:
            self.state_to('Error: '+str(exc))
        finally:
            if self.stop_event.is_set():
                self.state_to('Stopped')
