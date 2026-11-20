"""dVRK arms, ForceN tool IDs, device aliases, and topic names."""

import re


ARMS = ('PSM1', 'PSM2', 'PSM3')
MEASURED_CF = 'measured_cf'
SHAFT_FRAME = 'force_n_sensor'
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


def model_id(tool_type):
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


def shaft_namespace(arm):
    return f'/{arm}/shaft'


def tool_wrench_topic(model):
    """Relative raw-wrench topic for an already canonical model ID."""
    token = model.replace('[', '_rev_').rstrip(']')
    return f'tool_{token}/{MEASURED_CF}'
