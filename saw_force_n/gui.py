"""ATI-style Sensor Stats / Interval Stats tabs with hardware Rebias."""
from PyQt5 import QtWidgets, QtCore
from pathlib import Path


class Window(QtWidgets.QWidget):
    def __init__(self, driver, namespace='/'):
        super().__init__()
        self.driver = driver
        arm = next((part for part in namespace.split('/')
                    if part in ('PSM1', 'PSM2', 'PSM3')), None)
        device = Path(driver.port).name
        tool = device if device in ('CLASSIC_LND', 'S_LND', 'CLASSIC_MBF', 'S_MBF') else None
        identity = f"{arm or namespace} — {tool or device}"
        self.setWindowTitle(f'Forcen Force Sensor — {identity}')
        layout = QtWidgets.QVBoxLayout(self)
        self.identity = QtWidgets.QLabel(
            f"Arm: {arm or 'Unassigned'}    |    Tool: {tool or 'Unknown'}")
        self.identity.setTextFormat(QtCore.Qt.PlainText)
        font = self.identity.font()
        font.setBold(True)
        font.setPointSize(font.pointSize() + 2)
        self.identity.setFont(font)
        layout.addWidget(self.identity)
        self.connection = QtWidgets.QLabel(f'Device: {driver.port}\nNamespace: {namespace}')
        self.connection.setTextFormat(QtCore.Qt.PlainText)
        self.connection.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.connection.setWordWrap(True)
        layout.addWidget(self.connection)
        tabs = QtWidgets.QTabWidget()
        layout.addWidget(tabs)
        sensor = QtWidgets.QWidget()
        body = QtWidgets.QVBoxLayout(sensor)
        measurement_note = QtWidgets.QLabel(
            'Force is measured in newtons (N). Torque is not measured; displayed zeros are placeholders.')
        measurement_note.setWordWrap(True)
        body.addWidget(measurement_note)
        self.table = QtWidgets.QTableWidget(2, 3)
        self.table.setHorizontalHeaderLabels(['X', 'Y', 'Z'])
        self.table.setVerticalHeaderLabels(['Force [N]', 'Torque [N·m]'])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.table.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        body.addWidget(self.table)
        self.info = QtWidgets.QLabel('Connecting')
        self.info.setWordWrap(True)
        body.addWidget(self.info)
        self.button = QtWidgets.QPushButton('Zero force (hardware tare)')
        self.button.setToolTip('Set the current unloaded force readings to zero using the sensor’s hardware tare.')
        self.button.clicked.connect(self.rebias)
        body.addWidget(self.button)
        note = QtWidgets.QLabel(
            'Allow at least 5 minutes of warm-up before zeroing.\n'
            'Remove external loads and keep the tool still, then click “Zero force”.\n'
            'Zero again after changing the tool’s orientation or cycling sensor power.')
        note.setWordWrap(True)
        body.addWidget(note)
        body.addStretch()
        tabs.addTab(sensor, 'Sensor Stats')
        self.stats = QtWidgets.QLabel()
        self.stats.setMargin(15)
        tabs.addTab(self.stats, 'Interval Stats')
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        # Populate and style the widgets before calculating the initial size.
        self.refresh()
        self.table.resizeRowsToContents()
        self.table.setFixedHeight(
            self.table.horizontalHeader().sizeHint().height()
            + sum(self.table.rowHeight(row) for row in range(self.table.rowCount()))
            + 2 * self.table.frameWidth())
        layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.resize(self.sizeHint().expandedTo(QtCore.QSize(640, 460)))

    def rebias(self):
        self.driver.request_tare()
        self.refresh()

    def refresh(self):
        data = self.driver.snapshot()
        valid = data['state'] == 'Streaming' and data['age'] is not None and data['age'] < 2
        self.info.setText(data['state'])
        self.info.setStyleSheet('background: #b5e5b5; padding: 8px' if valid else
                               'background: #f0d0a0; padding: 8px')
        self.button.setEnabled(valid)
        for axis in range(3):
            text = f"{data['force'][axis]:.6f}" if valid and data['force'] else '—'
            self.table.setItem(0, axis, QtWidgets.QTableWidgetItem(text))
            self.table.setItem(1, axis, QtWidgets.QTableWidgetItem('0.000000'))
        self.stats.setText(f"Samples: {data['count']}\nReceive rate: {data['hz']:.1f} Hz\n"
                           f"Min/max receive interval: {data['minimum']*1000:.2f} / {data['maximum']*1000:.2f} ms\n"
                           f"Last sample age: {data['age'] if data['age'] is not None else 'N/A'} s")
