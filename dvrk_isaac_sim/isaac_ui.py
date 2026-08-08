"""Isaac Sim GUI for monitoring and commanding virtual CRTK arms."""

from __future__ import annotations

import math
from typing import Any


class IsaacCRTKWindow:
    """Live CRTK monitor and joint/state command window.

    This module is imported only for non-headless Isaac Sim runs.  It uses
    display units of degrees for revolute joints and millimetres for insertion
    joints, while commands sent to the model remain radians/metres.
    """

    _STATE_COMMANDS = ("enable", "disable", "pause", "resume", "home", "unhome", "fault", "clear_fault")

    def __init__(self, components: list[Any]) -> None:
        import omni.ui as ui

        self._ui = ui
        self._components = components
        self._fields: dict[str, list[Any]] = {}
        self._dirty: set[str] = set()
        self._refreshing = False
        self._refresh_counter = 0
        self._window = ui.Window("dVRK CRTK Monitor", width=520, height=760)
        with self._window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=8, height=0):
                    ui.Label("Virtual patient cart", style={"font_size": 20})
                    ui.Label("Measured joints: degrees / millimetres", style={"color": 0xFFB0B0B0})
                    for component in components:
                        self._add_component(component)
        self.update()

    def _add_component(self, component: Any) -> None:
        ui = self._ui
        name = component.config.name
        with ui.CollapsableFrame(name, collapsed=False):
            with ui.VStack(spacing=4, height=0):
                state_label = ui.Label("")
                homed_label = ui.Label("")
                ui.Label("Joint controls", style={"font_size": 15})
                fields = []
                for joint in component.config.joints:
                    with ui.HStack(height=26, spacing=5):
                        ui.Label(joint.name, width=100)
                        measured = ui.Label("", width=95)
                        model = ui.SimpleFloatModel(0.0)
                        field = ui.FloatField(model=model, width=105)
                        ui.Label("deg" if joint.type == "revolute" else "mm", width=35)
                        fields.append((joint, measured, model, field))
                        model.add_value_changed_fn(lambda _model, arm=name: self._mark_dirty(arm))
                with ui.HStack(spacing=5):
                    ui.Button("Apply joint targets", clicked_fn=lambda arm=name: self._apply_joints(arm))
                    ui.Button("Home", clicked_fn=lambda arm=name: self._command_state(arm, "home"))
                with ui.HStack(spacing=5):
                    ui.Label("Operating state", width=100)
                    state_model = ui.ComboBox(0, *self._STATE_COMMANDS)
                    state_model.model.add_item_changed_fn(
                        lambda _model, _item, arm=name, combo=state_model: self._combo_command(arm, combo)
                    )
                self._fields[name] = {
                    "state": state_label, "homed": homed_label, "joints": fields,
                }

    def _mark_dirty(self, name: str) -> None:
        if not self._refreshing:
            self._dirty.add(name)

    def _component(self, name: str) -> Any | None:
        return next((item for item in self._components if item.config.name == name), None)

    def _command_state(self, name: str, command: str) -> None:
        component = self._component(name)
        if component is not None:
            component.command_state(command)

    def _combo_command(self, name: str, combo: Any) -> None:
        # Isaac Sim omni.ui ComboBox value models expose the selected item
        # index. Convert it to the CRTK command string before dispatch.
        index = combo.model.get_item_value_model().as_int
        if 0 <= index < len(self._STATE_COMMANDS):
            self._command_state(name, self._STATE_COMMANDS[index])

    def _apply_joints(self, name: str) -> None:
        component = self._component(name)
        if component is None:
            return
        values = []
        for joint, _measured, model, _field in self._fields[name]["joints"]:
            value = model.as_float
            values.append(math.radians(value) if joint.type == "revolute" else value / 1000.0)
        if component.command_joint_position(values):
            self._dirty.discard(name)

    def update(self) -> None:
        """Refresh labels and target fields from the current simulation state."""
        self._refresh_counter += 1
        if self._refresh_counter % 6 != 0:
            return
        self._refreshing = True
        try:
            for component in self._components:
                name = component.config.name
                controls = self._fields.get(name)
                if controls is None:
                    continue
                controls["state"].text = f"State: {component.operating_state}"
                controls["homed"].text = f"Homed: {component.is_homed}"
                measured = component.model.measured_js()
                goal = component.model.goal_js()
                for index, (joint, measured_label, model, _field) in enumerate(controls["joints"]):
                    measured_value = float(measured.position[index])
                    goal_value = float(goal.position[index])
                    if joint.type == "revolute":
                        measured_value = math.degrees(measured_value)
                        goal_value = math.degrees(goal_value)
                        unit = "deg"
                    else:
                        measured_value *= 1000.0
                        goal_value *= 1000.0
                        unit = "mm"
                    measured_label.text = f"{measured_value:8.2f} {unit}"
                    if name not in self._dirty:
                        model.set_value(goal_value)
        finally:
            self._refreshing = False

    def close(self) -> None:
        self._window.visible = False
