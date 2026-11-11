from types import SimpleNamespace

import pytest

from saw_force_n import ports


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
