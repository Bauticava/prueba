#!/usr/bin/env python3
import sys
from dataclasses import dataclass
from functools import partial
from typing import Callable, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QWidget,
)

from entity_spawner.srv import LoadEntity, SpawnEntity, RemoveEntity


@dataclass
class PendingCall:
    future: Future
    callback: Callable[[Optional[object], Optional[BaseException]], None]


class EntitySpawnerClientNode(Node):
    """ROS 2 client node handling service calls to the entity_spawner package."""

    def __init__(self) -> None:
        super().__init__('entity_spawner_gui_client')
        self._load_client = self.create_client(LoadEntity, 'loadEntity')
        self._spawn_client = self.create_client(SpawnEntity, 'spawnEntity')
        self._remove_client = self.create_client(RemoveEntity, 'removeEntity')

    def load_entity(self, stl_path: str, entity_name: str) -> Future:
        request = LoadEntity.Request()
        request.stl_path = stl_path
        request.entity_name = entity_name
        return self._load_client.call_async(request)

    def spawn_entity(
        self,
        entity_name: str,
        frame_id: str,
        position: List[float],
        orientation_rpy: List[float],
        scale: float,
        add_to_planning_scene: bool,
        spawn_in_sim_world: bool,
        sim_world: str,
        sim_reference_frame: str,
    ) -> Future:
        request = SpawnEntity.Request()
        request.entity_name = entity_name
        request.frame_id = frame_id
        request.x, request.y, request.z = position
        request.roll, request.pitch, request.yaw = orientation_rpy
        request.scale = scale
        request.add_to_planning_scene = add_to_planning_scene
        request.spawn_in_sim_world = spawn_in_sim_world
        request.sim_world = sim_world
        request.sim_reference_frame = sim_reference_frame
        return self._spawn_client.call_async(request)
    
    def remove_entity(self, entity_name: str) -> Future:
        request = RemoveEntity.Request()
        request.entity_name = entity_name
        return self._remove_client.call_async(request)

    def load_service_ready(self) -> bool:
        return self._load_client.service_is_ready()

    def spawn_service_ready(self) -> bool:
        return self._spawn_client.service_is_ready()
    
    def remove_service_ready(self) -> bool:
        return self._remove_client.service_is_ready()


class EntitySpawnerMainWindow(QMainWindow):
    """Main GUI window for interacting with entity loading and spawning services."""

    def __init__(self, node: EntitySpawnerClientNode) -> None:
        super().__init__()
        self._node = node
        self._pending_calls: List[PendingCall] = []

        self.setWindowTitle('Entity Spawner GUI')

        self._status_label = QLabel()
        self._status_label.setObjectName('statusLabel')

        self._stl_path_edit = QLineEdit()
        self._stl_path_edit.setPlaceholderText('Path to .stl file')

        self._entity_name_edit = QLineEdit()
        self._entity_name_edit.setPlaceholderText('Entity name')

        browse_button = QPushButton('Browse...')
        browse_button.clicked.connect(self._browse_for_stl)

        load_button = QPushButton('Load Entity')
        load_button.clicked.connect(self._on_load_clicked)

        spawn_button = QPushButton('Spawn Entity')
        spawn_button.clicked.connect(self._on_spawn_clicked)

        self._spawn_name_edit = QLineEdit()
        self._spawn_name_edit.setPlaceholderText('Entity name to spawn')

        self._frame_edit = QLineEdit()
        self._frame_edit.setPlaceholderText('Frame ID (e.g. world)')

        self._position_edits = [QLineEdit() for _ in range(3)]
        for edit, placeholder in zip(self._position_edits, ('X', 'Y', 'Z')):
            edit.setPlaceholderText(placeholder)

        self._orientation_edits = [QLineEdit() for _ in range(3)]
        for edit, placeholder in zip(self._orientation_edits, ('Roll', 'Pitch', 'Yaw')):
            edit.setPlaceholderText(placeholder)

        self._scale_edit = QLineEdit()
        self._scale_edit.setPlaceholderText('Scale factor (1.0)')
        self._scale_edit.setText('1.0')

        self._planning_scene_checkbox = QCheckBox('Add to planning scene')
        self._planning_scene_checkbox.setChecked(True)

        self._spawn_sim_checkbox = QCheckBox('Spawn in simulation world')
        self._spawn_sim_checkbox.toggled.connect(self._on_spawn_sim_toggled)

        self._sim_world_edit = QLineEdit()
        self._sim_world_edit.setPlaceholderText('Simulation world (default)')
        self._sim_world_edit.setText('default')
        self._sim_world_edit.setEnabled(False)

        self._sim_reference_edit = QLineEdit()
        self._sim_reference_edit.setPlaceholderText('Simulation reference frame (world)')
        self._sim_reference_edit.setText('world')
        self._sim_reference_edit.setEnabled(False)

        self._remove_name_edit = QLineEdit()
        self._remove_name_edit.setPlaceholderText('Entity name to remove')
        
        remove_button = QPushButton('Remove Entity')
        remove_button.clicked.connect(self._on_remove_clicked)

        load_group = QGroupBox('Load Entity')
        load_layout = QGridLayout()
        load_layout.addWidget(QLabel('STL File'), 0, 0)
        load_layout.addWidget(self._stl_path_edit, 0, 1)
        load_layout.addWidget(browse_button, 0, 2)
        load_layout.addWidget(QLabel('Entity Name'), 1, 0)
        load_layout.addWidget(self._entity_name_edit, 1, 1, 1, 2)
        load_layout.addWidget(load_button, 2, 0, 1, 3)
        load_group.setLayout(load_layout)

        spawn_group = QGroupBox('Spawn Entity')
        spawn_layout = QGridLayout()
        spawn_layout.addWidget(QLabel('Entity Name'), 0, 0)
        spawn_layout.addWidget(self._spawn_name_edit, 0, 1, 1, 2)
        spawn_layout.addWidget(QLabel('Frame ID'), 1, 0)
        spawn_layout.addWidget(self._frame_edit, 1, 1, 1, 2)
        spawn_layout.addWidget(QLabel('Position (m)'), 2, 0)
        for col, edit in enumerate(self._position_edits, start=1):
            spawn_layout.addWidget(edit, 2, col)
        spawn_layout.addWidget(QLabel('Orientation (rad)'), 3, 0)
        for col, edit in enumerate(self._orientation_edits, start=1):
            spawn_layout.addWidget(edit, 3, col)
        spawn_layout.addWidget(QLabel('Scale'), 4, 0)
        spawn_layout.addWidget(self._scale_edit, 4, 1, 1, 2)
        spawn_layout.addWidget(self._planning_scene_checkbox, 5, 0, 1, 3)
        spawn_layout.addWidget(self._spawn_sim_checkbox, 6, 0, 1, 3)
        spawn_layout.addWidget(QLabel('Simulation World'), 7, 0)
        spawn_layout.addWidget(self._sim_world_edit, 7, 1, 1, 2)
        spawn_layout.addWidget(QLabel('Reference Frame'), 8, 0)
        spawn_layout.addWidget(self._sim_reference_edit, 8, 1, 1, 2)
        spawn_layout.addWidget(spawn_button, 9, 0, 1, 3)
        spawn_group.setLayout(spawn_layout)

        remove_group = QGroupBox('Remove Entity')
        remove_layout = QGridLayout()
        remove_layout.addWidget(QLabel('Entity Name'), 0, 0)
        remove_layout.addWidget(self._remove_name_edit, 0, 1)
        remove_layout.addWidget(remove_button, 1, 0, 1, 2)
        remove_group.setLayout(remove_layout)

        central_widget = QWidget()
        main_layout = QGridLayout()
        main_layout.addWidget(load_group, 0, 0)
        main_layout.addWidget(spawn_group, 1, 0)
        main_layout.addWidget(remove_group, 2, 0)
        main_layout.addWidget(self._status_label, 3, 0)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self._status_label.setText('Waiting for services...')

        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros)
        self._spin_timer.start(50)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._spin_timer.stop()
        super().closeEvent(event)

    def _browse_for_stl(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Select STL file', '', 'STL Files (*.stl);;All Files (*)')
        if path:
            self._stl_path_edit.setText(path)

    def _on_load_clicked(self) -> None:
        stl_path = self._stl_path_edit.text().strip()
        entity_name = self._entity_name_edit.text().strip()

        if not stl_path:
            self._set_status('Please select an STL file.', error=True)
            return
        if not entity_name:
            self._set_status('Please provide a unique entity name.', error=True)
            return
        if not self._node.load_service_ready():
            self._set_status('loadEntity service not available.', error=True)
            return

        future = self._node.load_entity(stl_path, entity_name)
        self._enqueue_future(future, partial(self._handle_service_response, action='Load'))
        self._set_status(f"Loading entity '{entity_name}'...")

    def _on_spawn_clicked(self) -> None:
        entity_name = self._spawn_name_edit.text().strip()
        frame_id = self._frame_edit.text().strip() or 'world'

        if not entity_name:
            self._set_status('Provide name of a loaded entity to spawn.', error=True)
            return
        if not self._node.spawn_service_ready():
            self._set_status('spawnEntity service not available.', error=True)
            return

        try:
            position = [float(edit.text().strip() or 0.0) for edit in self._position_edits]
            orientation = [float(edit.text().strip() or 0.0) for edit in self._orientation_edits]
        except ValueError:
            self._set_status('Position and orientation must be valid numbers.', error=True)
            return

        try:
            scale_value = float(self._scale_edit.text().strip() or 1.0)
        except ValueError:
            self._set_status('Scale must be a valid number.', error=True)
            return

        if scale_value <= 0.0:
            self._set_status('Scale must be greater than zero.', error=True)
            return

        add_to_planning_scene = self._planning_scene_checkbox.isChecked()
        spawn_in_sim = self._spawn_sim_checkbox.isChecked()
        if not add_to_planning_scene and not spawn_in_sim:
            add_to_planning_scene = True

        sim_world = self._sim_world_edit.text().strip() if spawn_in_sim else ''
        sim_reference = self._sim_reference_edit.text().strip() if spawn_in_sim else ''

        future = self._node.spawn_entity(
            entity_name,
            frame_id,
            position,
            orientation,
            scale_value,
            add_to_planning_scene,
            spawn_in_sim,
            sim_world,
            sim_reference,
        )
        self._enqueue_future(future, partial(self._handle_service_response, action='Spawn'))
        targets = []
        if add_to_planning_scene:
            targets.append(f"frame '{frame_id}'")
        if spawn_in_sim:
            targets.append(f"simulation world '{sim_world or 'default'}'")
        target_text = ' and '.join(targets) if targets else 'planning scene'
        self._set_status(
            f"Spawning entity '{entity_name}' in {target_text} with scale {scale_value:.3f}..."
        )

    def _on_remove_clicked(self) -> None:
        entity_name = self._remove_name_edit.text().strip()
        if not entity_name:
            self._set_status('Please provide the name of the entity to remove.', error=True)
            return
        
        if not self._node.remove_service_ready():
            self._set_status('removeEntity service not available.', error=True)
            return

        future = self._node.remove_entity(entity_name)
        self._enqueue_future(future, partial(self._handle_service_response, action='Remove'))
        self._set_status(f"Removing entity '{entity_name}'...")
    
    def _on_spawn_sim_toggled(self, checked: bool) -> None:
        self._sim_world_edit.setEnabled(checked)
        self._sim_reference_edit.setEnabled(checked)

    def _enqueue_future(
        self,
        future: Future,
        callback: Callable[[Optional[object], Optional[BaseException]], None],
    ) -> None:
        self._pending_calls.append(PendingCall(future=future, callback=callback))

    def _spin_ros(self) -> None:
        try:
            rclpy.spin_once(self._node, timeout_sec=0.01)
        except ExternalShutdownException:
            return

        finished: List[PendingCall] = [entry for entry in self._pending_calls if entry.future.done()]
        self._pending_calls = [entry for entry in self._pending_calls if not entry.future.done()]

        for entry in finished:
            exception = entry.future.exception()
            result = None
            if exception is None:
                result = entry.future.result()
            entry.callback(result, exception)

        if (self._node.load_service_ready() and self._node.spawn_service_ready() and self._node.remove_service_ready()):
            if self._status_label.text().startswith('Waiting for services'):
                self._set_status('Services ready.', error=False)

    def _handle_service_response(
        self,
        result: Optional[object],
        exception: Optional[BaseException],
        *,
        action: str,
    ) -> None:
        if exception is not None:
            self._set_status(f'{action} request failed: {exception}', error=True)
            return

        if hasattr(result, 'success') and hasattr(result, 'message'):
            if result.success:
                self._set_status(result.message or f'{action} succeeded.')
            else:
                self._set_status(result.message or f'{action} failed.', error=True)
        else:
            self._set_status(f'{action} response received.', error=False)

    def _set_status(self, message: str, *, error: bool = False) -> None:
        color = 'red' if error else 'black'
        self._status_label.setStyleSheet(f'color: {color};')
        self._status_label.setText(message)


def main() -> int:
    rclpy.init()
    node = EntitySpawnerClientNode()

    app = QApplication(sys.argv)
    window = EntitySpawnerMainWindow(node)
    window.show()

    exit_code = app.exec()

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
