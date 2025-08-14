#!/usr/bin/env python3
import sys
import threading
import select
import termios
import tty
import fcntl
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class _TTYReader:
    """Open a TTY for raw, non-blocking single-character reads."""
    def __init__(self):
        self._use_stdin = sys.stdin.isatty()
        self._fd = None
        self._fh = None
        self._old_attrs = None
        self._old_flags = None

    def open(self):
        if self._use_stdin:
            self._fd = sys.stdin.fileno()
            self._fh = None
        else:
            candidates = []
            env_tty = os.environ.get('TTY')
            if env_tty:
                candidates.append(env_tty)
            if hasattr(os, 'ctermid'):
                candidates.append(os.ctermid())
            candidates.append('/dev/tty')

            for path in candidates:
                try:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                    self._fd = fd
                    self._fh = os.fdopen(fd, 'rb', buffering=0)
                    break
                except Exception:
                    continue

            if self._fd is None:
                raise RuntimeError(
                    "No TTY available (stdin is not a TTY and /dev/tty could not be opened)."
                )

        # raw + non-blocking
        self._old_attrs = termios.tcgetattr(self._fd)
        tty.setraw(self._fd)
        self._old_flags = fcntl.fcntl(self._fd, fcntl.F_GETFL)
        fcntl.fcntl(self._fd, fcntl.F_SETFL, self._old_flags | os.O_NONBLOCK)

    def close(self):
        if self._fd is None:
            return
        try:
            if self._old_attrs is not None:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)
        except Exception:
            pass
        try:
            if self._old_flags is not None:
                fcntl.fcntl(self._fd, fcntl.F_SETFL, self._old_flags)
        except Exception:
            pass
        try:
            if self._fh is not None:
                self._fh.close()
        except Exception:
            pass
        self._fd = None
        self._fh = None

    def read_char(self):
        if self._fd is None:
            return None
        rlist, _, _ = select.select([self._fd], [], [], 0.05)
        if not rlist:
            return None
        try:
            b = os.read(self._fd, 1)
            if not b:
                return None
            return b.decode('utf-8', errors='ignore')
        except Exception:
            return None


class TeleopKeyboardNode(Node):
    """
    Keys:
      w: +linear x by linear_step
      s: -linear x by linear_step
      a: +angular z by rotational_step (turn left)
      d: -angular z by rotational_step (turn right)
      z: reset angular z to 0.0
      c: reset linear  x to 0.0
      Ctrl+C: quit  (NEW)
    """

    def __init__(self):
        super().__init__('trackbot_teleop_keyboard')

        # Parameters (YAML/CLI)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_rotational_speed', 1.0)
        self.declare_parameter('linear_step', 0.1)
        self.declare_parameter('rotational_step', 0.1)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_rotational_speed').value)
        self.step_lin = float(self.get_parameter('linear_step').value)
        self.step_ang = float(self.get_parameter('rotational_step').value)

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.lin_x = 0.0
        self.ang_z = 0.0

        period = 1.0 / max(1e-3, self.publish_rate)
        self.timer = self.create_timer(period, self._publish_twist)

        self._tty = _TTYReader()
        try:
            self._tty.open()
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().error(
                "Run from a real terminal (or SSH with -t). The node will publish zeros."
            )
            self._tty = None
        else:
            self._stop = threading.Event()
            self._kbd_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._kbd_thread.start()
            self._print_banner()

    def _print_banner(self):
        self.get_logger().info(
            "\nKeyboard teleop ready.\n"
            f"Publishing to: {self.cmd_vel_topic}\n"
            f"Max linear: {self.max_lin:.3f} m/s | Max angular: {self.max_ang:.3f} rad/s\n"
            f"Steps -> linear: {self.step_lin:.3f} | angular: {self.step_ang:.3f}\n\n"
            "Controls:\n"
            "  w: +linear x    s: -linear x\n"
            "  a: +angular z   d: -angular z\n"
            "  z: reset angular z to 0.0\n"
            "  c: reset linear  x to 0.0\n"
            "  Ctrl+C to quit\n"
        )

    def _publish_twist(self):
        msg = Twist()
        msg.linear.x = self.lin_x
        msg.angular.z = self.ang_z
        self.pub.publish(msg)

    def _log_state(self, reason: str):
        # NEW: one line per change so you always see the latest set speeds
        self.get_logger().info(
            f"{reason} -> lin_x={self.lin_x:+.3f} m/s, ang_z={self.ang_z:+.3f} rad/s"
        )

    def _keyboard_loop(self):
        while rclpy.ok() and not getattr(self, "_stop", threading.Event()).is_set():
            ch = self._tty.read_char() if self._tty else None
            if not ch:
                continue

            # Handle Ctrl+C explicitly in raw mode (NEW)
            if ch == '\x03':  # ETX
                self._log_state("Ctrl+C")
                rclpy.shutdown()
                break

            updated = False
            if ch == 'w':
                self.lin_x = clamp(self.lin_x + self.step_lin, -self.max_lin, self.max_lin)
                updated = True
                self._log_state("w")
            elif ch == 's':
                self.lin_x = clamp(self.lin_x - self.step_lin, -self.max_lin, self.max_lin)
                updated = True
                self._log_state("s")
            elif ch == 'a':
                self.ang_z = clamp(self.ang_z + self.step_ang, -self.max_ang, self.max_ang)
                updated = True
                self._log_state("a")
            elif ch == 'd':
                self.ang_z = clamp(self.ang_z - self.step_ang, -self.max_ang, self.max_ang)
                updated = True
                self._log_state("d")
            elif ch == 'z':
                self.ang_z = 0.0
                updated = True
                self._log_state("z (reset ang_z)")
            elif ch == 'c':
                self.lin_x = 0.0
                updated = True
                self._log_state("c (reset lin_x)")

            if updated:
                # Keep a terse status line too (optional)
                sys.stdout.write(f"\rlin_x: {self.lin_x:+.3f} m/s | ang_z: {self.ang_z:+.3f} rad/s    ")
                sys.stdout.flush()

    def destroy_node(self):
        try:
            if hasattr(self, "_stop"):
                self._stop.set()
            if hasattr(self, "_kbd_thread"):
                self._kbd_thread.join(timeout=0.2)
        except Exception:
            pass
        try:
            if self._tty:
                self._tty.close()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
