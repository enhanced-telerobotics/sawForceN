"""Tool matching and nonblocking ownership of the sensor reader processes."""

from collections import Counter
from dataclasses import dataclass
import os
import re
import signal
import subprocess
import sys
import time


TOOL_MODELS = ('400906', '400972', '420906', '400872')
DEFAULT_TOOL_MAP = [f'{model}={model}' for model in TOOL_MODELS]


def parse_tool_map(entries):
    """Map ForceN models, optionally with an exact revision, to /dev aliases."""
    result = {}
    for entry in entries:
        model, separator, alias = entry.partition('=')
        if (not separator or not re.fullmatch(r'[0-9]{3}[89][0-9]{2}(?:\[(?:0|[1-9][0-9]*)\])?', model)
                or not re.fullmatch(r'[A-Za-z0-9_]+', alias)):
            raise ValueError(f'Invalid tool_map entry: {entry!r}; expected 400872=400872')
        if model in result or alias in result.values():
            raise ValueError(f'Duplicate model or device alias in tool_map: {entry!r}')
        base = model.split('[')[0]
        if any(existing.split('[')[0] == base and ('[' not in model or '[' not in existing)
               for existing in result):
            raise ValueError(f'Cannot mix unversioned and revision-specific mappings for {base}')
        result[model] = alias
    return result


def tool_alias(tool_type, tool_map):
    # dVRK mtsToolList::Name emits NAME:MODEL[version_min..version_max].
    match = re.fullmatch(
        r'(?:[^:\[\]\s]+:)?([0-9]{6})(?:\[([0-9]+|[0-9]*\.\.[0-9]*)\])?',
        tool_type.strip())
    if not match:
        return None
    model, revision = match.groups()
    if revision in (None, '..'):
        return tool_map.get(model)
    if '..' in revision:
        lower, upper = revision.split('..')
        if not lower or not upper or int(lower) != int(upper):
            return None
        revision = lower
    # Explicit revisions require an exact mapping. In particular, an old
    # 400972[2..2] message must never open the first MBF's /dev/400972 alias.
    return tool_map.get(f'{model}[{int(revision)}]')


@dataclass
class Reader:
    arm: str
    process: object
    deadline: float = None
    stop_stage: int = 0


class ToolManager:
    """One child per sensor; a replacement must wait for the old child to exit."""

    def __init__(self, arms, tool_map, logger, gui=True, *,
                 popen=subprocess.Popen, clock=time.monotonic):
        self.tool_types = dict.fromkeys(arms, '')
        self.tool_map = tool_map
        self.logger = logger
        self.gui = gui
        self.popen = popen
        self.clock = clock
        self.readers = {}  # device alias -> Reader (including readers stopping)
        self.failed = set()
        self.conflicts = set()

    def update(self, arm, tool_type):
        self.tool_types[arm] = tool_type

    def _stop(self, alias, reader):
        if reader.deadline is None:
            self.logger.info(f'Stopping /dev/{alias} on /{reader.arm}/shaft')
            try:
                reader.process.send_signal(
                    signal.CTRL_BREAK_EVENT if os.name == 'nt' else signal.SIGINT)
            except ProcessLookupError:
                pass
            reader.deadline = self.clock() + 5.0

    def reconcile(self):
        assignments = {arm: tool_alias(value, self.tool_map)
                       for arm, value in self.tool_types.items()}
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
                                        'change the tool assignment or restart force_n_tool_manager to retry')
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
            command = [sys.executable, '-m', 'saw_force_n.node', '--port', f'/dev/{alias}']
            if not self.gui:
                command.append('--no-gui')
            command += ['--ros-args', '-r', f'__ns:=/{arm}/shaft']
            try:
                options = ({'creationflags': subprocess.CREATE_NEW_PROCESS_GROUP}
                           if os.name == 'nt' else {'start_new_session': True})
                process = self.popen(command, **options)
            except OSError as exc:
                self.logger.error(f'Cannot start /dev/{alias} on {arm}: {exc}')
                self.failed.add((alias, arm))
                continue
            self.readers[alias] = Reader(arm, process)
            self.logger.info(f'Started /dev/{alias} on /{arm}/shaft')

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
