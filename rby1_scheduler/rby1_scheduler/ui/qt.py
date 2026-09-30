"""Small PySide6/PyQt5 compatibility layer for the scheduler UI."""
try:
    from PySide6.QtCore import QRectF, QSize, Qt, QTimer
    from PySide6.QtGui import QColor, QPainter, QPen
    from PySide6.QtWidgets import (
        QApplication, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
        QLineEdit, QMainWindow, QPlainTextEdit, QProgressBar, QPushButton,
        QTabWidget, QTableWidget, QTableWidgetItem, QToolTip, QVBoxLayout,
        QWidget,
    )
    QT_BINDING = 'PySide6'
except ImportError:
    from PyQt5.QtCore import QRectF, QSize, Qt, QTimer
    from PyQt5.QtGui import QColor, QPainter, QPen
    from PyQt5.QtWidgets import (
        QApplication, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
        QLineEdit, QMainWindow, QPlainTextEdit, QProgressBar, QPushButton,
        QTabWidget, QTableWidget, QTableWidgetItem, QToolTip, QVBoxLayout,
        QWidget,
    )
    QT_BINDING = 'PyQt5'


def app_exec(app: QApplication) -> int:
    runner = getattr(app, 'exec', None)
    return int(runner()) if runner is not None else int(app.exec_())
