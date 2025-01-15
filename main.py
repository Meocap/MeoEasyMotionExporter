import enum
import math
import os
from typing import List, Optional

import mathutils
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QComboBox, QFileDialog, QMessageBox
)
from PySide6.QtCore import Qt
import sys
from fbx_converter import FBXController, FBXFrame
from meocap_sdk import FrameReader

reader = FrameReader()

POSE_ATTRS = [
    "pelvis",  # 骨盆
    "left_hip",  # 左大腿
    "right_hip",  # 右大腿
    "spine1",  # 第一腰椎骨
    "left_knee",  # 左膝盖
    "right_knee",  # 右膝盖
    "spine2",  # 第二腰椎骨
    "left_ankle",  # 左脚踝
    "right_ankle",  # 右脚踝
    "spine3",  # 第三腰椎骨
    "left_foot",  # 左脚
    "right_foot",  # 右脚
    "neck",  # 脖子
    "left_clavicle",  # 左肩胛骨
    "right_clavicle",  # 右肩胛骨
    "head",  # 头
    "left_shoulder",  # 左肩
    "right_shoulder",  # 右肩
    "left_elbow",  # 左肘
    "right_elbow",  # 右肘
    "left_wrist",  # 左手腕
    "right_wrist",  # 右手腕
    "left_hand",  # 左手
    "right_hand"  # 右手
]

SKEL_MIXAMO = [
    "mixamorig:Hips", "mixamorig:LeftUpLeg", "mixamorig:RightUpLeg",
    "mixamorig:Spine",  # 3
    "mixamorig:LeftLeg", "mixamorig:RightLeg",
    "mixamorig:Spine1",  # 6
    "mixamorig:LeftFoot",  # 7
    "mixamorig:RightFoot",  # 8
    "mixamorig:Spine2",  # 9,
    "mixamorig:LeftToeBase", "mixamorig:RightToeBase",
    "mixamorig:Neck",
    "mixamorig:LeftShoulder", "mixamorig:RightShoulder",
    "mixamorig:Head",
    "mixamorig:LeftArm", "mixamorig:RightArm",
    "mixamorig:LeftForeArm", "mixamorig:RightForeArm",
    "mixamorig:LeftHand", "mixamorig:RightHand"
]


def load_file(file_name: str) -> str:
    return os.path.join(os.path.dirname(__file__), file_name)
class ExportFBXType(enum.Enum):
    BinaryFBX = 1
    AsciiFBX = 2


class SkeletonType(enum.Enum):
    Mixamo = 1


def convert_to_fbx(recording_path: str, output_fbx_path: str, input_fbx_path: Optional[str] = None,
                   export_type: Optional[ExportFBXType] = None, skel_type: Optional[SkeletonType] = None):
    if input_fbx_path is None or input_fbx_path == '':
        input_fbx_path = load_file("data/performer.fbx")
    print(f"Converting to FBX with params: \n"
          f"Recording Path: {recording_path}\n"
          f"Output FBX Path: {output_fbx_path}\n"
          f"Input FBX Path: {input_fbx_path}\n"
          f"Export Type: {export_type}\n"
          f"Skeleton Type: {skel_type}")

    # Add actual implementation here
    rd = FrameReader()
    frames = rd.decode_recordings(recording_path)
    controller = FBXController()
    fbx_frames = []

    for frame in frames:
        quat = [getattr(frame.optimized_pose, attr) for attr in POSE_ATTRS]
        quat = [mathutils.Quaternion(mathutils.Vector([q.w, q.i, -q.k, q.j])) for q in quat]
        # quat = [mathutils.Quaternion(mathutils.Vector([1, 0, 0, 0])) for q in quat]
        euler = [q.to_euler() for q in quat]
        euler = [(e.x, e.y, e.z) for e in euler]
        # X = Left Y = backward Z=Up

        trans: (float, float, float) = (frame.translation.x, -frame.translation.z, frame.translation.y)
        fbx_frames.append(FBXFrame(euler, trans))

    if skel_type is None:
        skel_type = SkeletonType.Mixamo
    if export_type is None:
        export_type = ExportFBXType.BinaryFBX

    if input_fbx_path is None:
        input_fbx_path = "performer.fbx"

    joint_names = SKEL_MIXAMO
    match skel_type:
        case SkeletonType.Mixamo:
            joint_names = SKEL_MIXAMO

    fbx_type = "FBX binary (*.fbx)"
    match export_type:
        case ExportFBXType.BinaryFBX:
            fbx_type = "FBX binary (*.fbx)"
        case ExportFBXType.AsciiFBX:
            fbx_type = "FBX ascii (*.fbx)"
    controller.export_fbx(fbx_frames, input_fbx_path, output_fbx_path, joint_names, fbx_type)


# Main Window
class FBXConverterUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FBX Converter")
        self.setGeometry(200, 200, 800, 280)

        # Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layouts
        main_layout = QVBoxLayout()
        form_layout = QVBoxLayout()
        button_layout = QHBoxLayout()

        # Widgets
        self.recording_path_input = self._create_file_input("Recording Path / 录制文件路径 :",
                                                            "Meocap Recording Files (*.meorecording)")
        self.input_fbx_path_input = self._create_file_input("Input FBX Path (Optional) / 要添加动画的FBX文件 (可选) :",
                                                            "FBX Files (*.fbx)")

        self._input_fbx_label = QLabel()
        self._input_fbx_label.setText("默认情况下将使用我们的Mixamo的默认FBX模型，您也可以导入一个FBX，但是这个FBX必须是您所选择的骨架，"
                                      "目前【骨架重定向】功能尚未完善，因此可能当导入您自定义的fbx时，会出现滑步等问题")
        self._input_fbx_label.setWordWrap(True)

        self.export_type_combo = self._create_dropdown("Export Type / 导出FBX的格式 :", ExportFBXType)
        self.skel_type_combo = self._create_dropdown("Skeleton Type / 骨架类型 :", SkeletonType)

        self.convert_button = QPushButton("Convert to FBX / 转换到FBX")
        self.convert_button.clicked.connect(self.convert)

        # Assemble Layouts
        form_layout.addLayout(self.recording_path_input["layout"])
        form_layout.addLayout(self.input_fbx_path_input["layout"])
        form_layout.addWidget(self._input_fbx_label)
        form_layout.addLayout(self.export_type_combo["layout"])
        form_layout.addLayout(self.skel_type_combo["layout"])

        button_layout.addWidget(self.convert_button)

        main_layout.addLayout(form_layout)
        main_layout.addLayout(button_layout)
        central_widget.setLayout(main_layout)

    def _create_file_input(self, label_text, file_filter):
        layout = QHBoxLayout()
        label = QLabel(label_text)
        line_edit = QLineEdit()
        browse_button = QPushButton("Browse 选择")

        browse_button.clicked.connect(lambda: self._browse_file(line_edit, file_filter))

        layout.addWidget(label)
        layout.addWidget(line_edit)
        layout.addWidget(browse_button)

        return {"layout": layout, "line_edit": line_edit}

    def _browse_file(self, line_edit, _filter):
        file_dialog = QFileDialog()
        file_path, _ = file_dialog.getOpenFileName(self, "Select File", "", _filter)
        if file_path:
            line_edit.setText(file_path)

    def _create_dropdown(self, label_text, enum_class):
        layout = QHBoxLayout()
        label = QLabel(label_text)
        combo_box = QComboBox()
        for item in enum_class:
            combo_box.addItem(item.name, item)

        layout.addWidget(label)
        layout.addWidget(combo_box)

        return {"layout": layout, "combo_box": combo_box}

    def convert(self):
        recording_path = self.recording_path_input["line_edit"].text()
        input_fbx_path = self.input_fbx_path_input["line_edit"].text()

        export_type = self.export_type_combo["combo_box"].currentData()
        skel_type = self.skel_type_combo["combo_box"].currentData()

        if not recording_path:
            QMessageBox.warning(self, "Missing Fields", "Recording Path is required.")
            return

        file_dialog = QFileDialog()
        output_fbx_path, _ = file_dialog.getSaveFileName(self, "Select Output FBX Path", "", "FBX Files (*.fbx)")
        if not output_fbx_path:
            QMessageBox.warning(self, "Missing Fields", "Output FBX Path is required.")
            return

        try:
            convert_to_fbx(recording_path, output_fbx_path, input_fbx_path, export_type, skel_type)
            QMessageBox.information(self, "Success", "FBX conversion completed successfully!")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"An error occurred: {e}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # 检测命令行参数并解析
        try:
            recording_path = sys.argv[1]
            output_fbx_path = sys.argv[2]
            input_fbx_path = None
            export_type = None
            skel_type = None
            # 调用转换函数
            convert_to_fbx(recording_path, output_fbx_path, input_fbx_path, export_type, skel_type)
            sys.exit(0)
        except IndexError:
            print(
                "Usage: python script.py <recording_path> <output_fbx_path> [<input_fbx_path>] [<export_type>] [<skel_type>]")
        finally:
            sys.exit(1)

    else:
        # 启动 UI
        app = QApplication(sys.argv)
        window = FBXConverterUI()
        window.show()
        sys.exit(app.exec())