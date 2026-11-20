"""Network definitions for the portable format-version-1 checkpoints."""

import torch
from torch import nn


class FCNModel(nn.Module):
    def __init__(self, width, hidden_size, num_layers, dropout):
        super().__init__()
        if num_layers < 1:
            raise ValueError('FCN num_layers must be at least 1')
        layers = []
        for _ in range(num_layers - 1):
            layers.extend([nn.Linear(width, hidden_size), nn.ReLU(), nn.Dropout(dropout)])
            width = hidden_size
        self.backbone = nn.Sequential(*layers) if layers else nn.Identity()
        self.head = nn.Linear(width, 3)

    def forward(self, x):
        return self.head(self.backbone(x[:, -1]))


class PositionalEncoding(nn.Module):
    def __init__(self, width):
        super().__init__()
        pe = torch.zeros(500, width)
        position = torch.arange(500).unsqueeze(1).float()
        div_term = torch.exp(torch.arange(0, width, 2).float()
                             * (-torch.log(torch.tensor(10000.0)) / width))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer('pe', pe.unsqueeze(0))

    def forward(self, x):
        return x + self.pe[:, :x.size(1)]


class TransformerModel(nn.Module):
    def __init__(self, width, hidden_size, nhead, num_layers):
        super().__init__()
        self.input_proj = nn.Linear(width, hidden_size)
        self.pos_encoder = PositionalEncoding(hidden_size)
        layer = nn.TransformerEncoderLayer(
            hidden_size, nhead, dim_feedforward=256, dropout=0.1,
            activation='relu', batch_first=True,
        )
        self.transformer_encoder = nn.TransformerEncoder(layer, num_layers=num_layers)
        self.head = nn.Sequential(
            nn.Linear(hidden_size, hidden_size), nn.ReLU(), nn.Linear(hidden_size, 3),
        )

    def forward(self, x):
        return self.head(self.transformer_encoder(self.pos_encoder(self.input_proj(x)))[:, -1])
