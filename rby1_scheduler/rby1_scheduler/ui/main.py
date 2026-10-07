"""PyQt scheduler UI entry point."""
from __future__ import annotations
import signal
import sys
import rclpy
from .main_window import SchedulerMainWindow
from .qt import QApplication, QT_BINDING, QTimer, app_exec
from .ros_client import SchedulerUiNode

def _run(argv) -> int:
    rclpy.init(args=argv[1:])
    app = QApplication(argv)
    app.setApplicationName('RB-Y1 Scheduler UI')
    node = SchedulerUiNode()
    window = SchedulerMainWindow(node.send_command)
    ros_timer = QTimer()
    ros_timer.setInterval(10)
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start()
    refresh_timer = QTimer()
    refresh_timer.setInterval(100)

    def refresh() -> None:
        state = node.consume_state()
        if state is not None:
            window.update_state(state)
        window.append_events(node.consume_events())

    refresh_timer.timeout.connect(refresh)
    refresh_timer.start()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    window.statusBar().showMessage(
        f'Qt: {QT_BINDING} | Space: Pause and cancel the active task'
    )
    window.show()
    try:
        return app_exec(app)
    finally:
        ros_timer.stop()
        refresh_timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None) -> None:
    del args
    raise SystemExit(_run(sys.argv))

if __name__ == '__main__':
    main()
