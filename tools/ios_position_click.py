"""Click calibrated controls in the IOS Tool window."""

from __future__ import annotations

import argparse
import ctypes
import subprocess
import sys
import time
from ctypes import wintypes
from pathlib import Path

user32 = ctypes.WinDLL("user32", use_last_error=True)
shell32 = ctypes.WinDLL("shell32", use_last_error=True)


def enable_dpi_awareness() -> None:
    try:
        user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
    except (AttributeError, OSError):
        user32.SetProcessDPIAware()
    try:
        user32.SetThreadDpiAwarenessContext(ctypes.c_void_p(-4))
    except AttributeError:
        pass


enable_dpi_awareness()


WINDOW_TITLE = "IOS Tool"
REFERENCE_SIZE = (1631, 1371)

# Coordinates are relative to the full IOS Tool window captured at REFERENCE_SIZE.
POSITIONS = {
    "2800": (619, 542),
    "left": (709, 374),
    "right": (724, 718),
    "flight-freeze": (1375, 182),
    "position": (171, 114),
    "environ": (562, 120),
    "windshear": (448, 187),
}

ALIASES = {
    "2800": "2800",
    "center": "2800",
    "centre": "2800",
    "left": "left",
    "l": "left",
    "right": "right",
    "r": "right",
    "flight-freeze": "flight-freeze",
    "freeze": "flight-freeze",
    "position": "position",
    "position-tab": "position",
    "environ": "environ",
    "environment": "environ",
    "windshear": "windshear",
    "wind-shear": "windshear",
}

SEQUENCES = {
    "approach-2800": (
        ("position", 0.5),
        ("2800", 0.8),
        ("flight-freeze", 20.0),
    ),
}

SW_RESTORE = 9
GA_ROOT = 2
WM_MOUSEMOVE = 0x0200
WM_LBUTTONDOWN = 0x0201
WM_LBUTTONUP = 0x0202
MK_LBUTTON = 0x0001
MOUSEEVENTF_LEFTDOWN = 0x0002
MOUSEEVENTF_LEFTUP = 0x0004
dwmapi = ctypes.WinDLL("dwmapi", use_last_error=True)
EnumWindowsProc = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
DWMWA_EXTENDED_FRAME_BOUNDS = 9

user32.WindowFromPoint.argtypes = [wintypes.POINT]
user32.WindowFromPoint.restype = wintypes.HWND
user32.GetAncestor.argtypes = [wintypes.HWND, wintypes.UINT]
user32.GetAncestor.restype = wintypes.HWND
shell32.ShellExecuteW.restype = ctypes.c_void_p


def is_elevated() -> bool:
    return bool(shell32.IsUserAnAdmin())


def relaunch_elevated() -> None:
    script = str(Path(sys.argv[0]).resolve())
    parameters = subprocess.list2cmdline([script, *sys.argv[1:]])
    result = shell32.ShellExecuteW(
        None,
        "runas",
        sys.executable,
        parameters,
        str(Path.cwd()),
        1,
    )
    code = ctypes.cast(result, ctypes.c_void_p).value or 0
    if code <= 32:
        raise ctypes.WinError(ctypes.get_last_error())
    print("Administrator relaunch requested; approve the Windows UAC prompt.")


def find_window(title: str) -> int:
    matches: list[int] = []

    @EnumWindowsProc
    def callback(hwnd: int, _lparam: int) -> bool:
        if not user32.IsWindowVisible(hwnd):
            return True
        length = user32.GetWindowTextLengthW(hwnd)
        if length == 0:
            return True
        buffer = ctypes.create_unicode_buffer(length + 1)
        user32.GetWindowTextW(hwnd, buffer, len(buffer))
        if buffer.value == title:
            matches.append(hwnd)
        return True

    if not user32.EnumWindows(callback, 0):
        raise ctypes.WinError(ctypes.get_last_error())
    if len(matches) != 1:
        raise RuntimeError(f"Expected one {title!r} window, found {len(matches)}")
    return matches[0]


def window_rect(hwnd: int) -> tuple[int, int, int, int]:
    rect = wintypes.RECT()
    result = dwmapi.DwmGetWindowAttribute(
        hwnd,
        DWMWA_EXTENDED_FRAME_BOUNDS,
        ctypes.byref(rect),
        ctypes.sizeof(rect),
    )
    if result != 0 and not user32.GetWindowRect(hwnd, ctypes.byref(rect)):
        raise ctypes.WinError(ctypes.get_last_error())
    return rect.left, rect.top, rect.right, rect.bottom


def scaled_screen_point(
    rect: tuple[int, int, int, int], point: tuple[int, int]
) -> tuple[int, int]:
    left, top, right, bottom = rect
    width = right - left
    height = bottom - top
    ref_width, ref_height = REFERENCE_SIZE
    x = left + round(point[0] * width / ref_width)
    y = top + round(point[1] * height / ref_height)
    return x, y


def message_click_screen_point(hwnd: int, x: int, y: int) -> None:
    user32.ShowWindow(hwnd, SW_RESTORE)
    user32.SetForegroundWindow(hwnd)
    time.sleep(0.25)

    screen_point = wintypes.POINT(x, y)
    target = user32.WindowFromPoint(screen_point)
    if not target or user32.GetAncestor(target, GA_ROOT) != hwnd:
        raise RuntimeError("The target pixel is not inside the IOS Tool window")

    client_point = wintypes.POINT(x, y)
    if not user32.ScreenToClient(target, ctypes.byref(client_point)):
        raise ctypes.WinError(ctypes.get_last_error())
    lparam = (client_point.y & 0xFFFF) << 16 | (client_point.x & 0xFFFF)
    user32.PostMessageW(target, WM_MOUSEMOVE, 0, lparam)
    user32.PostMessageW(target, WM_LBUTTONDOWN, MK_LBUTTON, lparam)
    time.sleep(0.05)
    user32.PostMessageW(target, WM_LBUTTONUP, 0, lparam)


def physical_click_screen_point(hwnd: int, x: int, y: int) -> None:
    user32.ShowWindow(hwnd, SW_RESTORE)
    user32.SetForegroundWindow(hwnd)
    time.sleep(0.25)
    if not user32.SetCursorPos(x, y):
        raise ctypes.WinError(ctypes.get_last_error())
    actual = wintypes.POINT()
    user32.GetCursorPos(ctypes.byref(actual))
    if (actual.x, actual.y) != (x, y):
        raise RuntimeError(
            f"Cursor coordinate mismatch: requested {(x, y)}, got {(actual.x, actual.y)}"
        )
    user32.mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0)
    time.sleep(0.05)
    user32.mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Click a calibrated control in the running IOS Tool window."
    )
    parser.add_argument(
        "actions",
        nargs="+",
        choices=sorted(ALIASES.keys() | SEQUENCES.keys()),
        help="one or more calibrated IOS controls, or approach-2800",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.4,
        help="seconds to wait before clicking (default: 0.4)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="find the window and print the target without clicking",
    )
    parser.add_argument(
        "--physical",
        action="store_true",
        help="physical click (kept for compatibility; this is now the default)",
    )
    parser.add_argument(
        "--message",
        action="store_true",
        help="use a window message instead of the default physical click",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.dry_run and not is_elevated():
        relaunch_elevated()
        return

    actions: list[tuple[str, float]] = []
    for action in args.actions:
        if action in SEQUENCES:
            actions.extend(SEQUENCES[action])
        else:
            actions.append((ALIASES[action], max(0.0, args.delay)))

    for index, (action, step_delay) in enumerate(actions, start=1):
        hwnd = find_window(WINDOW_TITLE)
        rect = window_rect(hwnd)
        x, y = scaled_screen_point(rect, POSITIONS[action])
        print(
            f"step={index}/{len(actions)} hwnd={hwnd} rect={rect} "
            f"action={action} target=({x}, {y}) delay={step_delay}"
        )
        if args.dry_run:
            continue

        time.sleep(step_delay)
        if args.message:
            message_click_screen_point(hwnd, x, y)
            method = "window-message"
        else:
            physical_click_screen_point(hwnd, x, y)
            method = "setcursorpos"
        print(f"clicked action={action} method={method}")


if __name__ == "__main__":
    main()
