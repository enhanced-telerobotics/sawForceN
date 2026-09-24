import threading
from saw_force_n.driver import Driver
from saw_force_n.protocol import Framer, parse_sample
import pytest


def test_fragmentation():
    f = Framer()
    assert f.feed(b'<1000 -20') == []
    assert parse_sample(f.feed(b'00 3000>')[0]) == [1000, -2000, 3000]


def test_tare_waits_for_completion():
    d = Driver('unused')
    class Serial:
        def write(self, data):
            sent.append(data)
    sent = []
    d.serial = Serial()
    responses = iter([[b'e0x1'], [b'e0x1'], [b'e0x4'], [b'e0x1']])
    d.receive = lambda: next(responses)
    d.state_to('Streaming')
    assert d.request_tare()
    assert not d.request_tare()
    assert d.take_samples() == []
    d.tare()
    assert sent == [b'<SDM0>', b'<SDM3>', b'<SDM2>']
    assert d.snapshot()['state'] == 'Streaming'


def test_tare_failure_does_not_restart():
    d = Driver('unused')
    sent = []
    class Serial:
        def write(self, data):
            sent.append(data)
    d.serial = Serial()
    responses = iter([[b'e0x1'], [b'e0x99']])
    d.receive = lambda: next(responses)
    with pytest.raises(RuntimeError):
        d.tare()
    assert b'<SDM2>' not in sent
    assert d.take_samples() == []
