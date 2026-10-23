"""pyserial 3.5 on Windows: spurious ReadFile/WriteFile errors under thread load.

serial.win32 binds kernel32 through a plain WinDLL and serialwin32 calls GetLastError() as a separate ctypes call
right after ReadFile/WriteFile. Between the two the thread can give up and take back the GIL, whose timed wait
leaves ERROR_TIMEOUT (1460) or 0 in the thread's last error: read() and write() then raise "ReadFile failed" /
"WriteFile failed" on a healthy port. Seen with Python 3.14 in Flux Studio (GUI, receiver and sender threads).

Importing this module rebinds those functions with use_last_error=True, so ctypes keeps the error of each call
itself, and points serial.win32.GetLastError at ctypes.get_last_error. Import it before opening a port."""
import sys


def _patch():
    if sys.platform != "win32":
        return
    import ctypes
    from serial import win32
    if getattr(win32, "_last_error_fixed", False):
        return
    k32 = ctypes.WinDLL("kernel32", use_last_error=True)
    for name in dir(win32):
        f = getattr(win32, name)
        if name == "GetLastError" or not isinstance(f, ctypes._CFuncPtr) or not hasattr(k32, name):
            continue
        g = getattr(k32, name)
        g.argtypes, g.restype = f.argtypes, f.restype
        setattr(win32, name, g)
    win32.GetLastError = ctypes.get_last_error
    win32._last_error_fixed = True


_patch()
