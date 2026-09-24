"""ATI-style Sensor Stats / Interval Stats tabs with hardware Rebias."""
from PyQt5 import QtWidgets, QtCore


class Window(QtWidgets.QWidget):
    def __init__(self, driver):
        super().__init__()
        self.driver = driver
        self.setWindowTitle('Forcen Force Sensor (N, N·m)')
        layout = QtWidgets.QVBoxLayout(self)
        tabs = QtWidgets.QTabWidget()
        layout.addWidget(tabs)
        sensor = QtWidgets.QWidget()
        body = QtWidgets.QVBoxLayout(sensor)
        body.addWidget(QtWidgets.QLabel('Calibrated forces [N]. Torque [N·m] is zero (not measured).'))
        self.table = QtWidgets.QTableWidget(2, 3)
        self.table.setHorizontalHeaderLabels(['X', 'Y', 'Z'])
        self.table.setVerticalHeaderLabels(['Force [N]', 'Torque [N·m]'])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        body.addWidget(self.table)
        self.info = QtWidgets.QLabel('Connecting')
        self.info.setWordWrap(True)
        body.addWidget(self.info)
        self.button = QtWidgets.QPushButton('Rebias (hardware tare)')
        self.button.clicked.connect(self.rebias)
        body.addWidget(self.button)
        note = QtWidgets.QLabel('Before taring: unload the sensor and keep it stationary.\n'
                               'Manual §9 recommends 5 minutes warm-up and re-taring after orientation changes.')
        body.addWidget(note)
        tabs.addTab(sensor, 'Sensor Stats')
        self.stats = QtWidgets.QLabel()
        self.stats.setMargin(15)
        tabs.addTab(self.stats, 'Interval Stats')
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.resize(600, 350)

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
