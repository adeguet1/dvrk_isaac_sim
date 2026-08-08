"""CRTK operating-state state machine."""

from __future__ import annotations


class CRTKOperatingState:
    """Small, deterministic state machine for the CRTK operating-state API."""

    DISABLED = "DISABLED"
    ENABLED = "ENABLED"
    PAUSED = "PAUSED"
    FAULT = "FAULT"

    def __init__(self, initial_state: str = DISABLED) -> None:
        self.state = initial_state
        self.is_homed = True

    @property
    def accepts_motion(self) -> bool:
        return self.state == self.ENABLED

    def command(self, command: str) -> tuple[bool, str]:
        """Apply a CRTK ``state_command`` and return success and an error."""
        command = str(command).strip().lower()
        if command == "enable":
            if self.state == self.FAULT:
                return False, "cannot enable while in FAULT; issue clear_fault first"
            self.state = self.ENABLED
        elif command == "disable":
            self.state = self.DISABLED
        elif command == "pause":
            if self.state != self.ENABLED:
                return False, f"cannot pause from {self.state}"
            self.state = self.PAUSED
        elif command == "resume":
            if self.state != self.PAUSED:
                return False, f"cannot resume from {self.state}"
            self.state = self.ENABLED
        elif command == "home":
            self.is_homed = True
        elif command == "unhome":
            self.is_homed = False
        elif command == "fault":
            self.state = self.FAULT
        elif command in ("clear_fault", "reset"):
            if self.state != self.FAULT:
                return False, f"cannot clear fault from {self.state}"
            self.state = self.DISABLED
        else:
            return False, f"unknown state command {command!r}"
        return True, ""

