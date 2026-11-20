"""Load a trained shaft-force model and run causal inference on the CPU."""

from collections import deque

import numpy as np
import torch
from torch import nn

from .models import FCNModel, TransformerModel
from .tool_types import tool_model


class ForceModel(nn.Module):
    """Residual regressor with training normalization and an affine baseline."""

    def __init__(self, metadata):
        super().__init__()
        width = len(metadata['input_columns'])
        for key, size in (('input_mean', width), ('input_std', width),
                          ('target_mean', 3), ('target_std', 3)):
            self.register_buffer(key, torch.zeros(size))
        self.register_buffer('affine_weights', torch.zeros(width + 1, 3))

        config = metadata['model_config']
        kind = config['model_type']
        if kind == 'fcn':
            self.network = FCNModel(width, config['hidden_size'], config['num_layers'],
                                    config.get('dropout', 0.1))
        elif kind == 'transformer':
            self.network = TransformerModel(width, config['hidden_size'],
                                            config.get('nhead', 4), config['num_layers'])
        elif kind == 'ridge':
            self.network = None
        else:
            raise ValueError(f'Unsupported checkpoint model_type: {kind}')

    def forward(self, x):
        x = (x - self.input_mean) / self.input_std
        baseline = x[:, -1] @ self.affine_weights[:-1] + self.affine_weights[-1]
        residual = self.network(x) if self.network is not None else 0.0
        return (baseline + residual) * self.target_std + self.target_mean


class ForcePredictor:
    def __init__(self, checkpoint_path, expected_model):
        checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=True)
        if checkpoint.get('format_version') != 1:
            raise ValueError('Expected a portable format-version-1 force checkpoint')
        self.metadata = metadata = checkpoint['metadata']
        declared_tools = metadata.get('tool_types') or metadata.get('tool_type') or []
        if isinstance(declared_tools, str):
            declared_tools = [declared_tools]
        if expected_model not in {tool_model(value) for value in declared_tools}:
            raise ValueError(f'Checkpoint tool metadata {declared_tools!r} does not match {expected_model}')
        if metadata['target_frame'] != 'shaft' or metadata['target_units'] != 'N':
            raise ValueError('Expected a shaft-frame force checkpoint in newtons')
        target_columns = ('ati_fx', 'ati_fy', 'ati_fz')
        if tuple(metadata.get('target_columns', target_columns)) != target_columns:
            raise ValueError('Expected force outputs in x, y, z order')
        self.input_columns = tuple(metadata['input_columns'])
        if not self.input_columns or len(set(self.input_columns)) != len(self.input_columns):
            raise ValueError('Checkpoint input_columns must be nonempty and unique')
        self.sequence_length = metadata['sequence_length']
        self.sample_period_s = float(metadata['sample_period_s'])
        if not isinstance(self.sequence_length, int) or self.sequence_length < 1:
            raise ValueError('Checkpoint sequence_length must be a positive integer')
        if metadata['model_config']['model_type'] == 'transformer' and self.sequence_length > 500:
            raise ValueError('Transformer sequence length exceeds its positional encoding')
        if not np.isfinite(self.sample_period_s) or self.sample_period_s <= 0:
            raise ValueError('Checkpoint sample_period_s must be positive and finite')

        self.model = ForceModel(metadata)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.eval()
        if (not all(torch.isfinite(value).all() for value in self.model.state_dict().values())
                or torch.any(self.model.input_std <= 0) or torch.any(self.model.target_std <= 0)):
            raise ValueError('Checkpoint has invalid model weights or normalization')

        self._constant_indices = np.empty(0, dtype=np.int64)
        self._constant_values = np.empty(0)
        if 'input_min' in metadata or 'input_max' in metadata:
            minimum = np.asarray(metadata.get('input_min'), dtype=np.float64)
            maximum = np.asarray(metadata.get('input_max'), dtype=np.float64)
            if (minimum.shape != (len(self.input_columns),) or maximum.shape != minimum.shape
                    or not np.isfinite(minimum).all() or not np.isfinite(maximum).all()
                    or np.any(minimum > maximum)):
                raise ValueError('Checkpoint feature bounds do not match input_columns')
            self._constant_indices = np.flatnonzero(minimum == maximum)
            self._constant_values = minimum[self._constant_indices]
        self._history = deque(maxlen=self.sequence_length)
        self._last_timestamp = None

    def reset(self):
        self._history.clear()
        self._last_timestamp = None

    @torch.inference_mode()
    def predict(self, features, timestamp_s):
        features = np.asarray(features, dtype=np.float32)
        if features.shape != (len(self.input_columns),) or not np.isfinite(features).all():
            self.reset()
            raise ValueError(f'Expected {len(self.input_columns)} finite feature values')
        # Constant channels such as the raw torque placeholders have no trained
        # response and can be amplified by the normalization floor.
        if np.any(np.abs(features[self._constant_indices] - self._constant_values) > 1e-6):
            self.reset()
            raise ValueError('An input changed from its constant training value')
        if not np.isfinite(timestamp_s):
            self.reset()
            raise ValueError('Invalid inference timestamp')
        if self._last_timestamp is not None:
            elapsed = timestamp_s - self._last_timestamp
            if elapsed <= 0 or elapsed > 2.5 * self.sample_period_s:
                self.reset()
        self._last_timestamp = timestamp_s
        self._history.append(features.copy())
        if len(self._history) < self.sequence_length:
            return None
        x = torch.from_numpy(np.stack(self._history))[None]
        prediction = self.model(x)[0].numpy()
        if not np.isfinite(prediction).all():
            self.reset()
            raise RuntimeError('Model returned nonfinite force')
        return prediction
