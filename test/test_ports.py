from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from saw_force_n import driver, ports


def sensor(device='COM12', serial='2084358A4242'):
    return SimpleNamespace(device=device, vid=0x0483, pid=0x5740,
                           serial_number=serial)


def test_windows_alias_follows_usb_identity(monkeypatch):
    monkeypatch.setattr(ports, 'os', SimpleNamespace(name='nt'))
    devices = [sensor('COM3', 'different'), sensor('COM12')]
    monkeypatch.setattr(ports.list_ports, 'comports', lambda: devices)
    assert ports.resolve_port('/dev/400906') == 'COM12'
    assert ports.resolve_port('400906') == 'COM12'
    assert ports.resolve_port('COM25') == 'COM25'
    assert ports.default_port() == '/dev/400906'


@pytest.mark.parametrize('devices', [[], [sensor('COM3'), sensor('COM4')]])
def test_windows_requires_unique_tool(monkeypatch, devices):
    monkeypatch.setattr(ports, 'os', SimpleNamespace(name='nt'))
    monkeypatch.setattr(ports.list_ports, 'comports', lambda: devices)
    with pytest.raises(ValueError, match='expected one USB sensor'):
        ports.resolve_port('/dev/400906')


def test_windows_rejects_linux_device_path(monkeypatch):
    monkeypatch.setattr(ports, 'os', SimpleNamespace(name='nt'))
    with pytest.raises(ValueError, match='Linux device path'):
        ports.resolve_port('/dev/ttyACM0')


def test_linux_preserves_device_paths(monkeypatch):
    monkeypatch.setattr(ports, 'os', SimpleNamespace(name='posix'))
    assert ports.resolve_port('/dev/420906') == '/dev/420906'
    assert ports.resolve_port('/dev/ttyACM0') == '/dev/ttyACM0'
    assert ports.default_port() == '/dev/ttyACM0'


@pytest.mark.parametrize('model, serial', [
    ('400906', '2084358A4242'), ('420906', '206B35894242'),
    ('400972', '209F35894242'), ('400872', '205D358E4242'),
])
def test_windows_model_aliases(monkeypatch, model, serial):
    monkeypatch.setattr(ports, 'os', SimpleNamespace(name='nt'))
    monkeypatch.setattr(ports.list_ports, 'comports', lambda: [sensor(serial=serial)])
    assert ports.resolve_port('/dev/' + model) == 'COM12'
    assert ports.resolve_port(model) == 'COM12'


def test_windows_driver_reads_from_resolved_com_port(monkeypatch):
    monkeypatch.setattr(ports, 'os', SimpleNamespace(name='nt'))
    monkeypatch.setattr(ports.list_ports, 'comports', lambda: [sensor()])
    worker = driver.Driver('/dev/400906')
    connection = MagicMock()
    connection.__enter__.return_value = connection
    connection.in_waiting = 64
    sample = b'<1000 -2000 3000>'
    replies = iter([b'<e0x1>', b'<r0x123>', *([b'<e0x1>'] * 4), sample])

    def read(_size):
        reply = next(replies)
        if reply == sample:
            worker.stop_event.set()
        return reply

    connection.read.side_effect = read
    open_serial = MagicMock(return_value=connection)
    monkeypatch.setattr(driver.serial, 'Serial', open_serial)
    worker.run()

    open_serial.assert_called_once_with('COM12', 115200, timeout=.1, write_timeout=1, exclusive=True)
    assert worker.snapshot()['count'] == 1
    assert worker.snapshot()['state'] == 'Stopped'
    connection.write.assert_called_with(b'<SDM0>')
    connection.__exit__.assert_called_once_with(None, None, None)
