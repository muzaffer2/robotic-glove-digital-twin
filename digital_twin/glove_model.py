"""PyBullet 3D virtual model of the textile-based pneumatic robotic glove."""

from __future__ import annotations

from pathlib import Path
from typing import Dict

from .pneumatics import FingerKinematics

try:
    import pybullet as p
    import pybullet_data

    HAS_PYBULLET = True
except ImportError:
    p = None  # type: ignore
    pybullet_data = None  # type: ignore
    HAS_PYBULLET = False


FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
DEFAULT_URDF_PATH = Path(__file__).resolve().parent / "assets" / "robotic_glove.urdf"

DEFAULT_CAMERA = {
    "distance": 0.34,
    "yaw": 38.0,
    "pitch": -18.0,
    "target": [0.0, 0.0, 0.11],
}


class PyBulletGloveTwin:
    """Loads the 5-finger robotic glove URDF in PyBullet and synchronizes joint angles."""

    def __init__(self, gui: bool = True, urdf_path: Path | None = None) -> None:
        if not HAS_PYBULLET:
            raise RuntimeError("pybullet is not installed")

        self.gui = gui
        self.urdf_path = urdf_path or DEFAULT_URDF_PATH
        self.client_id = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setGravity(0, 0, 0, physicsClientId=self.client_id)

        self._hud_text_id: int = -1
        self._hint_text_id: int = -1
        self._last_hud_str: str = ""
        self._last_color_intensity: Dict[str, float] = {}

        # Load reference ground plane for spatial orientation when orbiting/zooming
        try:
            self.plane_id = p.loadURDF(
                "plane.urdf",
                basePosition=[0.0, 0.0, 0.0],
                physicsClientId=self.client_id,
            )
            p.changeVisualShape(
                self.plane_id,
                -1,
                rgbaColor=[0.18, 0.21, 0.26, 1.0],
                physicsClientId=self.client_id,
            )
        except Exception:
            self.plane_id = -1

        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
            self.reset_camera()
            self._hint_text_id = p.addUserDebugText(
                "Space/R: Reset View | 1:Front 2:Side 3:Top | +/-: Zoom | Mouse: Orbit",
                [-0.14, 0.0, 0.015],
                textColorRGB=[0.70, 0.82, 0.95],
                textSize=1.1,
                physicsClientId=self.client_id,
            )

        self.robot_id = p.loadURDF(
            str(self.urdf_path),
            basePosition=[0.0, 0.0, 0.055],
            useFixedBase=True,
            physicsClientId=self.client_id,
        )

        self.joints: Dict[str, int] = {}
        self.links: Dict[str, int] = {}
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for idx in range(num_joints):
            info = p.getJointInfo(self.robot_id, idx, physicsClientId=self.client_id)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            self.joints[joint_name] = idx
            self.links[link_name] = idx

    def reset_camera(self, preset: str = "default") -> None:
        """Reset or switch PyBullet camera to a clean preset view."""
        if not p.isConnected(self.client_id):
            return
        presets = {
            "default": (DEFAULT_CAMERA["distance"], DEFAULT_CAMERA["yaw"], DEFAULT_CAMERA["pitch"]),
            "front": (0.34, 0.0, -8.0),
            "side": (0.34, 90.0, -8.0),
            "top": (0.36, 0.0, -85.0),
        }
        dist, yaw, pitch = presets.get(preset, presets["default"])
        p.resetDebugVisualizerCamera(
            cameraDistance=dist,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=list(DEFAULT_CAMERA["target"]),
            physicsClientId=self.client_id,
        )

    def adjust_zoom(self, delta: float) -> None:
        """Zoom camera in/out around the current target."""
        if not p.isConnected(self.client_id):
            return
        cam = p.getDebugVisualizerCamera(physicsClientId=self.client_id)
        yaw, pitch, dist, target = cam[8], cam[9], cam[10], cam[11]
        new_dist = max(0.14, min(1.2, dist + delta))
        p.resetDebugVisualizerCamera(
            cameraDistance=new_dist,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=list(target),
            physicsClientId=self.client_id,
        )

    def handle_keyboard_events(self) -> None:
        """Poll PyBullet window keyboard shortcuts (Space/R reset, 1/2/3 presets, +/- zoom)."""
        if not self.gui or not p.isConnected(self.client_id):
            return
        keys = p.getKeyboardEvents(physicsClientId=self.client_id)
        for key, state in keys.items():
            if not (state & p.KEY_WAS_TRIGGERED):
                continue
            if key in (ord(" "), ord("r"), ord("R")):
                self.reset_camera("default")
            elif key == ord("1"):
                self.reset_camera("front")
            elif key == ord("2"):
                self.reset_camera("side")
            elif key == ord("3"):
                self.reset_camera("top")
            elif key in (ord("+"), ord("=")):
                self.adjust_zoom(-0.04)
            elif key in (ord("-"), ord("_")):
                self.adjust_zoom(0.04)

    def set_finger_kinematics(
        self,
        finger: str,
        kinematics: FingerKinematics,
        pressure_kpa: float = 0.0,
    ) -> None:
        """Apply MCP/PIP/DIP angles and dorsal actuator color feedback to a finger."""
        for suffix, angle in (
            ("mcp", kinematics.mcp_rad),
            ("pip", kinematics.pip_rad),
            ("dip", kinematics.dip_rad),
        ):
            jname = f"{finger}_{suffix}"
            if jname in self.joints:
                p.resetJointState(
                    self.robot_id,
                    self.joints[jname],
                    targetValue=angle,
                    physicsClientId=self.client_id,
                )

        # Color dorsal pneumatic bellow (shapeIndex=1) smoothly with pressure (0..160 kPa)
        intensity = round(max(0.0, min(1.0, pressure_kpa / 160.0)), 2)
        if self._last_color_intensity.get(finger) != intensity:
            self._last_color_intensity[finger] = intensity
            rgba = [0.18 + 0.77 * intensity, 0.58 - 0.18 * intensity, 0.95 - 0.70 * intensity, 1.0]
            for seg in ("proximal", "middle"):
                lname = f"{finger}_{seg}"
                if lname in self.links:
                    p.changeVisualShape(
                        self.robot_id,
                        self.links[lname],
                        shapeIndex=1,
                        rgbaColor=rgba,
                        physicsClientId=self.client_id,
                    )

    def update_from_pressures(
        self,
        finger_pressures: Dict[str, FingerKinematics],
        active_kpa: float,
        mirror_all: bool = False,
    ) -> None:
        """Update mapped fingers, process camera shortcuts, and step simulation."""
        self.handle_keyboard_events()
        for finger, kin in finger_pressures.items():
            p_kpa = active_kpa if (mirror_all or finger == "index") else 0.0
            self.set_finger_kinematics(finger, kin, pressure_kpa=p_kpa)

        if self.gui:
            idx_kin = finger_pressures.get("index")
            curl = idx_kin.total_curl_deg if idx_kin else 0.0
            hud_str = f"P_s1: {active_kpa:6.1f} kPa  |  Curl: {curl:5.1f} deg"
            if hud_str != self._last_hud_str:
                self._last_hud_str = hud_str
                kwargs = {
                    "text": hud_str,
                    "textPosition": [-0.09, 0.0, 0.255],
                    "textColorRGB": [0.35, 0.85, 1.0],
                    "textSize": 1.3,
                    "physicsClientId": self.client_id,
                }
                if self._hud_text_id >= 0:
                    kwargs["replaceItemUniqueId"] = self._hud_text_id
                self._hud_text_id = p.addUserDebugText(**kwargs)

        p.stepSimulation(physicsClientId=self.client_id)

    def get_joint_angles_deg(self, finger: str) -> tuple[float, float, float]:
        """Read back current (MCP, PIP, DIP) angles in degrees from PyBullet."""
        import math

        vals = []
        for suffix in ("mcp", "pip", "dip"):
            idx = self.joints[f"{finger}_{suffix}"]
            state = p.getJointState(self.robot_id, idx, physicsClientId=self.client_id)
            vals.append(math.degrees(state[0]))
        return vals[0], vals[1], vals[2]

    def close(self) -> None:
        if p.isConnected(self.client_id):
            p.disconnect(self.client_id)
