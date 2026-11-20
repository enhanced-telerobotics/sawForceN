"""Shared dVRK model IDs and ForceN topic names."""

import re


def tool_model(tool_type):
    # dVRK mtsToolList::Name emits NAME:MODEL[version_min..version_max].
    match = re.fullmatch(
        r'(?:[^:\[\]\s]+:)?([0-9]{6})(?:\[([0-9]+|[0-9]*\.\.[0-9]*)\])?',
        tool_type.strip())
    if not match:
        return None
    model, revision = match.groups()
    if revision in (None, '..'):
        return model
    if '..' in revision:
        lower, upper = revision.split('..')
        if not lower or not upper or int(lower) != int(upper):
            return None
        revision = lower
    # Explicit revisions require an exact mapping. In particular, an old
    # 400972[2..2] message must never open the first MBF's /dev/400972 alias.
    return f'{model}[{int(revision)}]'



def tool_wrench_topic(model):
    """Relative raw-wrench topic for an already canonical model ID."""
    token = model.replace('[', '_rev_').rstrip(']')
    return f'tool_{token}/measured_cf'
