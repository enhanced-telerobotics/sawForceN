"""Tool matching and nonblocking ownership of the sensor reader processes."""

from collections import Counter
from dataclasses import dataclass
import signal
import subprocess
import sys
import time

from .tools import shaft_namespace


@dataclass
class Reader:
    arm: str
    process: object
    deadline: float = None
    stop_stage: int = 0


class Readers:
    """One child per sensor; a replacement must wait for the old child to exit."""

    def __init__(self, tool_map, logger, gui=True, *,
                 popen=subprocess.Popen, clock=time.monotonic):
        self.tool_map = tool_map
        self.logger = logger
        self.gui = gui
        self.popen = popen
        self.clock = clock
        self.readers = {}  # device alias -> Reader (including readers stopping)
        self.failed = set()
        self.conflicts = set()

    def _stop(self, alias, reader):
        if reader.deadline is None:
            self.logger.info(f'Stopping /dev/{alias} on {shaft_namespace(reader.arm)}')
            try:
                reader.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            reader.deadline = self.clock() + 5.0

    def reconcile(self, models):
        assignments = {arm: self.tool_map.get(model) for arm, model in models.items()}
        counts = Counter(alias for alias in assignments.values() if alias)
        conflicts = {alias for alias, count in counts.items() if count > 1}
        for alias in sorted(conflicts - self.conflicts):
            arms = ', '.join(arm for arm, value in assignments.items() if value == alias)
            self.logger.error(f'{arms} report the same sensor /dev/{alias}; '
                              'no reader will run for it until the conflict is resolved')
        self.conflicts = conflicts
        desired = {alias: arm for arm, alias in assignments.items()
                   if alias and alias not in conflicts}
        self.failed.intersection_update((alias, arm) for alias, arm in desired.items())

        for alias, reader in list(self.readers.items()):
            code = reader.process.poll()
            if code is not None:
                if reader.deadline is None:
                    self.logger.warning(f'/dev/{alias} reader on {reader.arm} exited ({code}); '
                                        'change the tool assignment or restart force_publisher to retry')
                    self.failed.add((alias, reader.arm))
                del self.readers[alias]
                continue
            if desired.get(alias) != reader.arm:
                self._stop(alias, reader)
            if reader.deadline is not None and self.clock() >= reader.deadline:
                try:
                    if reader.stop_stage == 0:
                        self.logger.warning(f'/dev/{alias} reader did not stop; terminating')
                        reader.process.terminate()
                        reader.stop_stage = 1
                    else:
                        reader.process.kill()
                        reader.stop_stage = 2
                except ProcessLookupError:
                    pass
                reader.deadline = self.clock() + 2.0

        for alias, arm in desired.items():
            # Also wait for a different sensor to release this arm's namespace.
            if (alias in self.readers or (alias, arm) in self.failed
                    or any(reader.arm == arm for reader in self.readers.values())):
                continue
            command = [sys.executable, '-m', 'saw_force_n.sensor', '--port', f'/dev/{alias}',
                       '--tool-model', models[arm]]
            if not self.gui:
                command.append('--no-gui')
            namespace = shaft_namespace(arm)
            command += ['--ros-args', '-r', f'__ns:={namespace}']
            try:
                process = self.popen(command, start_new_session=True)
            except OSError as exc:
                self.logger.error(f'Cannot start /dev/{alias} on {arm}: {exc}')
                self.failed.add((alias, arm))
                continue
            self.readers[alias] = Reader(arm, process)
            self.logger.info(f'Started /dev/{alias} on {namespace}')

    def close(self):
        for alias, reader in self.readers.items():
            if reader.process.poll() is None:
                self._stop(alias, reader)
        for reader in self.readers.values():
            try:
                reader.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                reader.process.terminate()
                try:
                    reader.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    reader.process.kill()
                    reader.process.wait()
        self.readers.clear()
