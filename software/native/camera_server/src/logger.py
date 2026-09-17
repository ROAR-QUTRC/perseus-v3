from typing import Literal

# Color code definitions
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
BOLD = "\033[1m"
RESET = "\033[0m"

# debug is opt in when enable_debug is called
show_debug_logs = False


def enable_debug():
    global show_debug_logs
    show_debug_logs = True


def log(message: str, level: Literal["DEBUG", "INFO", "WARN", "ERROR"] = "INFO"):
    if level == "DEBUG" and not show_debug_logs:
        return  # Skip debug messages if debug logging is not enabled
    color = BLUE
    match level:
        case "DEBUG":
            color = GREEN
        case "INFO":
            color = BLUE
        case "WARN":
            color = YELLOW
        case "ERROR":
            color = RED
    print(f"{color}[Cam Server {BOLD}{level}{RESET}{color}]{RESET} {message}")
