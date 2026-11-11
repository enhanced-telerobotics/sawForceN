"""Resolve the lab's Linux tool aliases to Windows USB serial ports."""
import os

from serial.tools import list_ports


# Keep these identities in sync with udev/99-forcen.rules.
TOOL_SERIALS = {
    '400906': '2084358A4242',
    '420906': '206B35894242',
    '400972': '209F35894242',
    '400872': '205D358E4242',
}


def default_port():
    return '/dev/400906' if os.name == 'nt' else '/dev/ttyACM0'


def resolve_port(port):
    """Keep explicit ports; on Windows find exactly one matching lab tool."""
    if os.name != 'nt':
        return port
    tool = port.removeprefix('/dev/')
    if tool not in TOOL_SERIALS:
        if port.startswith('/dev/'):
            raise ValueError(f'{port} is a Linux device path; use a Windows COM port or lab tool alias')
        return port
    matches = [p.device for p in list_ports.comports()
               if p.vid == 0x0483 and p.pid == 0x5740
               and p.serial_number == TOOL_SERIALS[tool]]
    if len(matches) != 1:
        raise ValueError(
            f'{tool}: expected one USB sensor with serial {TOOL_SERIALS[tool]}, '
            f'found {len(matches)}. Connect the tool or specify --port COM<number>.')
    return matches[0]
