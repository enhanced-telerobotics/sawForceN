"""Assignment and process lifecycle tests, without ROS or sensor hardware."""

import signal
from types import SimpleNamespace
import subprocess
from unittest.mock import Mock

import pytest

from saw_force_n import tool_manager
from saw_force_n.tool_manager import ToolManager, parse_tool_map, tool_alias


@pytest.mark.parametrize('value', [
    '400906', 'LND_FORCEN:400906', 'LND_FORCEN:400906[..]',
    ' LND_FORCEN:400906[..] ',
])
def test_custom_model(value):
    assert tool_alias(value, {'400906': 'SENSOR_LND'}) == 'SENSOR_LND'


@pytest.mark.parametrize('value', [
    '', 'ERROR', 'LND:400006[..]', 'LND_FORCEN:400907[..]',
    'LND_FORCEN:400906extra', 'LND_FORCEN:400906[broken]',
])
def test_unregistered_or_invalid_tool(value):
    assert tool_alias(value, {'400906': 'SENSOR_LND'}) is None


@pytest.mark.parametrize('value, expected', [
    ('MBF_FORCEN:400972[1]', '400172_v1'),
    ('MBF_FORCEN:400972[1..1]', '400172_v1'),
    ('MBF_FORCEN:400972[2]', '400172_v2'),
    ('MBF_FORCEN:400972[2..2]', '400172_v2'),
    ('400972[2]', '400172_v2'),
    ('MBF_FORCEN:400972', None),
    ('MBF_FORCEN:400972[..]', None),
    ('MBF_FORCEN:400972[1..2]', None),
    ('MBF_FORCEN:400972[2..]', None),
    ('MBF_FORCEN:400972[..2]', None),
    ('MBF_FORCEN:400972[3]', None),
    ('MBF_FORCEN:400972[2..1]', None),
])
def test_physical_tools_with_same_model(value, expected):
    mapping = parse_tool_map(['400972[1]=400172_v1', '400972[2]=400172_v2'])
    assert tool_alias(value, mapping) == expected


@pytest.mark.parametrize('value, expected', [
    ('MBF_FORCEN:400972', '400972'),
    ('MBF_FORCEN:400972[..]', '400972'),
    ('MBF_FORCEN:400872', '400872'),
    ('MBF_FORCEN:400872[..]', '400872'),
    ('400872', '400872'),
    ('MBF_FORCEN:420972[..]', None),
    ('MBF_FORCEN:400972[1]', None),
    ('MBF_FORCEN:400972[2]', None),
    ('MBF_FORCEN:400972[1..1]', None),
    ('MBF_FORCEN:400972[2..2]', None),
    ('MBF_FORCEN:400972[1..2]', None),
])
def test_flat_models_and_obsolete_instance_ids(value, expected):
    mapping = parse_tool_map(['400972=400972', '400872=400872'])
    assert tool_alias(value, mapping) == expected


@pytest.mark.parametrize('entries', [
    ['400006=SENSOR_LND'], ['400772=400172E'], ['400872=400172[2]'], ['400906=/dev/SENSOR_LND'], ['400906=../ttyACM0'],
    ['400906=SENSOR_LND', '400906=OTHER'],
    ['400906=SENSOR_LND', '400907=SENSOR_LND'],
    ['400972=400172', '400972[1]=400172_v1'],
    ['400972[1]=400172_v1', '400972=400172'],
    ['400972[1]=400172_v1', '400972[2]=400172_v1'],
    ['400972[1..2]=400172'], ['400972[01]=400172'],
])
def test_invalid_configuration(entries):
    with pytest.raises(ValueError):
        parse_tool_map(entries)


@pytest.fixture
def manager(monkeypatch):
    monkeypatch.setattr(tool_manager, 'os', SimpleNamespace(name='posix'))
    processes = []

    def popen(command, **kwargs):
        process = Mock()
        process.poll.return_value = None
        process.command = command
        processes.append(process)
        return process

    result = ToolManager(['PSM1', 'PSM2', 'PSM3'],
                         {'400906': 'SENSOR_LND', '400907': 'OTHER'},
                         Mock(), gui=False, popen=Mock(side_effect=popen), clock=Mock(return_value=0))
    result.processes = processes
    return result


def select(manager, arm='PSM1', value='LND_FORCEN:400906[..]'):
    manager.update(arm, value)
    manager.reconcile()


def test_start_and_repeated_event(manager):
    select(manager)
    process = manager.processes[0]
    assert process.command[-1] == '__ns:=/PSM1/shaft'
    assert '/dev/SENSOR_LND' in process.command
    assert '--no-gui' in process.command
    select(manager)
    assert len(manager.processes) == 1


def test_move_waits_for_old_process_exit(manager):
    select(manager)
    old = manager.processes[0]
    manager.update('PSM1', '')
    select(manager, 'PSM3')
    old.send_signal.assert_called_once_with(signal.SIGINT)
    assert len(manager.processes) == 1
    old.poll.return_value = 0
    manager.reconcile()
    assert len(manager.processes) == 2
    assert manager.processes[-1].command[-1] == '__ns:=/PSM3/shaft'


def test_replacing_tool_waits_for_namespace(manager):
    select(manager)
    old = manager.processes[0]
    select(manager, value='OTHER:400907[..]')
    assert len(manager.processes) == 1
    old.poll.return_value = 0
    manager.reconcile()
    assert '/dev/OTHER' in manager.processes[-1].command


def test_duplicate_tool_stops_until_conflict_resolves(manager):
    select(manager)
    old = manager.processes[0]
    select(manager, 'PSM2')
    old.send_signal.assert_called_once_with(signal.SIGINT)
    old.poll.return_value = 0
    manager.reconcile()
    assert not manager.readers
    manager.update('PSM1', '')
    manager.reconcile()
    assert manager.processes[-1].command[-1] == '__ns:=/PSM2/shaft'


def test_two_distinct_tools(manager):
    select(manager)
    select(manager, 'PSM2', 'OTHER:400907[..]')
    assert len(manager.readers) == 2


def test_two_revisions_can_run_together_and_conflicts_are_per_sensor(manager):
    manager.tool_map = parse_tool_map(['400972[1]=400172_v1', '400972[2]=400172_v2'])
    select(manager, 'PSM1', 'MBF_FORCEN:400972[1..1]')
    select(manager, 'PSM2', 'MBF_FORCEN:400972[2..2]')
    assert set(manager.readers) == {'400172_v1', '400172_v2'}
    assert '/dev/400172_v1' in manager.processes[0].command
    assert '/dev/400172_v2' in manager.processes[1].command
    select(manager, 'PSM3', 'MBF_FORCEN:400972[1..1]')
    manager.processes[0].send_signal.assert_called_once_with(signal.SIGINT)
    manager.processes[1].send_signal.assert_not_called()
    assert manager.conflicts == {'400172_v1'}


@pytest.mark.parametrize('value', ['', 'ERROR', 'LND:400006[..]'])
def test_removal_or_non_forcen_tool_stops_reader(manager, value):
    select(manager)
    select(manager, value=value)
    manager.processes[0].send_signal.assert_called_once_with(signal.SIGINT)


def test_stuck_process_escalates_before_replacement(manager):
    select(manager)
    old = manager.processes[0]
    manager.update('PSM1', '')
    select(manager, 'PSM2')
    manager.clock.return_value = 5
    manager.reconcile()
    old.terminate.assert_called_once()
    manager.clock.return_value = 7
    manager.reconcile()
    old.kill.assert_called_once()
    assert len(manager.processes) == 1
    old.poll.return_value = -9
    manager.reconcile()
    assert len(manager.processes) == 2


def test_unexpected_exit_does_not_respawn_loop(manager):
    select(manager)
    manager.processes[0].poll.return_value = 1
    manager.reconcile()
    manager.reconcile()
    assert len(manager.processes) == 1
    select(manager, value='')
    select(manager)
    assert len(manager.processes) == 2


def test_spawn_failure_can_retry_after_assignment_change(manager):
    manager.popen.side_effect = OSError('Cannot execute')
    select(manager)
    manager.reconcile()
    assert manager.popen.call_count == 1
    assert not manager.readers
    select(manager, value='')
    select(manager)
    assert manager.popen.call_count == 2


def test_shutdown_reaps_children(manager):
    select(manager)
    process = manager.processes[0]
    process.wait.side_effect = [subprocess.TimeoutExpired('reader', 5), None]
    manager.close()
    process.send_signal.assert_called_once_with(signal.SIGINT)
    process.terminate.assert_called_once()
    assert not manager.readers


def test_windows_reader_process_group_and_shutdown(manager, monkeypatch):
    monkeypatch.setattr(tool_manager, 'os', SimpleNamespace(name='nt'))
    monkeypatch.setattr(subprocess, 'CREATE_NEW_PROCESS_GROUP', 512, raising=False)
    monkeypatch.setattr(signal, 'CTRL_BREAK_EVENT', 1, raising=False)
    select(manager)
    assert manager.popen.call_args.kwargs == {'creationflags': 512}
    select(manager, value='')
    manager.processes[0].send_signal.assert_called_once_with(signal.CTRL_BREAK_EVENT)
