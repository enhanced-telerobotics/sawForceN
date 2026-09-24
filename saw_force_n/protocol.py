"""ASCII framing from the validated Ubuntu reader; MAN-00017-001."""
import math

class Framer:
    """Handle fragmented, concatenated frames and discard noise with bounded memory."""
    def __init__(self):
        self.buffer = bytearray()
        self.start = None
        self.discarded = 0

    def feed(self, data):
        frames = []
        for byte in data:
            if byte in (ord('U'), ord('<')):
                if self.start is not None:
                    self.discarded += 1
                self.start = byte
                self.buffer.clear()
            elif self.start is not None:
                end = ord('Z') if self.start == ord('U') else ord('>')
                if byte == end:
                    frames.append(bytes(self.buffer))
                    self.start = None
                    self.buffer.clear()
                elif len(self.buffer) >= 8192:
                    self.discarded += 1
                    self.start = None
                    self.buffer.clear()
                else:
                    self.buffer.append(byte)
        return frames


def parse_sample(payload):
    try:
        tokens = payload.replace(b'\x00', b'').decode('ascii').split()
        if not tokens:
            return None
        values = [float(t) for t in tokens]
        return values if all(math.isfinite(v) for v in values) else None
    except (UnicodeDecodeError, ValueError):
        return None

