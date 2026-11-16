"""Independent thread watchdog; real hardware still needs an external fail-off path."""
import threading
import time


class WatchdogMotor:
    def __init__(self, driver, timeout_s=.25):
        self.driver, self.timeout = driver, timeout_s
        self.lock = threading.Lock()
        self.armed = False
        self.deadline = 0.
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._watch, daemon=True, name="motor-watchdog")
        self.thread.start()

    def arm(self):
        with self.lock:
            self.driver.stop()
            self.armed, self.deadline = True, time.monotonic()+self.timeout

    def apply(self, command):
        with self.lock:
            if not self.armed or time.monotonic() >= self.deadline:
                self.armed = False
                self.driver.stop()
                raise RuntimeError("Motor watchdog expired or not armed; new command required")
            self.driver.apply(command)
            self.deadline = time.monotonic()+self.timeout

    def stop(self):
        with self.lock:
            self.armed = False
            self.driver.stop()

    def _watch(self):
        while not self.closed.wait(min(.02, self.timeout/4)):
            with self.lock:
                if self.armed and time.monotonic() >= self.deadline:
                    self.armed = False
                    self.driver.stop()

    def close(self):
        self.stop()
        self.closed.set()
        self.thread.join(timeout=1.)
