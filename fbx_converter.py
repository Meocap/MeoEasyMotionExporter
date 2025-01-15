import math
from typing import List, Tuple
import fbx
import dataclasses
import mathutils


@dataclasses.dataclass
class FBXFrame:
    rots: List[Tuple[float, float, float]]
    trans: Tuple[float, float, float]


class FBXController:
    def __init__(self):
        self.import_fbx_path = "./data/performer.fbx"
        self.export_fbx_path = "output.fbx"
        self.joints = []
        self.total_frame = 0
        self.fbx_type = "FBX binary (*.fbx)"

    def export_fbx(self, frames: List[FBXFrame], import_fbx_path, export_fbx_path,
                   joint_names, fbx_type):

        self.joints = joint_names
        self.fbx_type = fbx_type
        self.total_frame = len(frames)
        self.import_fbx_path = import_fbx_path
        self.export_fbx_path = export_fbx_path

        # Initialize the FBX manager
        fbx_manager = fbx.FbxManager.Create()

        # Set IO settings
        io_settings = fbx.FbxIOSettings.Create(fbx_manager, "IOSROOT")
        fbx_manager.SetIOSettings(io_settings)

        # Create scene and importer
        fbx_scene = fbx.FbxScene.Create(fbx_manager, "myScene")
        fbx_importer = fbx.FbxImporter.Create(fbx_manager, "Importer")

        # Import the FBX scene
        if not fbx_importer.Initialize(self.import_fbx_path, -1, io_settings):
            print("Error:", fbx_importer.GetStatus().GetErrorString())
            return

        # Import the scene
        fbx_importer.Import(fbx_scene)

        # Set scene info
        scene_info = fbx.FbxDocumentInfo.Create(fbx_manager, "SceneInfo")
        scene_info.mTitle = "myScene"
        scene_info.mSubject = "mySceneAKA"
        scene_info.mAuthor = "Shutong"
        scene_info.mRevision = "1.0"
        scene_info.mKeywords = "myScene"
        scene_info.mComment = "myScene"

        fbx_scene.SetSceneInfo(scene_info)

        # Add animation
        self.add_animation(fbx_scene, frames)

        # Save the scene
        self.save_scene(fbx_manager, fbx_scene, self.export_fbx_path)

        # Clean up
        fbx_importer.Destroy()
        fbx_manager.Destroy()

    def save_scene(self, fbx_manager, fbx_scene, write_path):
        # Export FBX
        fbx_exporter = fbx.FbxExporter.Create(fbx_manager, "Exporter")
        io_settings = fbx.FbxIOSettings.Create(fbx_manager, "IOSROOT")
        fbx_manager.SetIOSettings(io_settings)

        file_format = fbx_manager.GetIOPluginRegistry().FindWriterIDByDescription(self.fbx_type)
        io_settings.SetBoolProp(fbx.EXP_FBX_MATERIAL, True)
        io_settings.SetBoolProp(fbx.EXP_FBX_TEXTURE, True)
        io_settings.SetBoolProp(fbx.EXP_FBX_EMBEDDED, True)
        io_settings.SetBoolProp(fbx.EXP_FBX_SHAPE, True)
        io_settings.SetBoolProp(fbx.EXP_FBX_GOBO, True)
        io_settings.SetBoolProp(fbx.EXP_FBX_ANIMATION, True)
        io_settings.SetBoolProp(fbx.EXP_FBX_GLOBAL_SETTINGS, True)

        if not fbx_exporter.Initialize(write_path, file_format, fbx_manager.GetIOSettings()):
            print("Error:", fbx_exporter.GetStatus().GetErrorString())
            return

        fbx_exporter.Export(fbx_scene)
        fbx_exporter.Destroy()

    def add_animation(self, fbx_scene, frames: List[FBXFrame]):

        # Root node for animation
        fbx_root_node = fbx_scene.GetRootNode()

        rest_local_rots = []

        def fbxrot2quat(rot):
            return mathutils.Euler(
                [rot[0] * math.pi / 180.0, rot[1] * math.pi / 180.0, rot[2] * math.pi / 180.0]).to_quaternion()

        def tuple2quat(rot: (float, float, float)):
            return mathutils.Euler(
                [rot[0], rot[1], rot[2]]).to_quaternion()

        trans_offset = (0, 0, 0)

        for joint in self.joints:
            fbx_node = fbx_root_node.FindChild(joint)
            if fbx_node:
                if joint == self.joints[0]:
                    translation = fbx_node.LclTranslation.Get()
                    trans_offset = (translation[0], translation[1], translation[2])
                rest_local_rots.append(fbxrot2quat(fbx_node.LclRotation.Get()))
            else:
                rest_local_rots.append(mathutils.Quaternion([1, 0, 0, 0]))
        rest_global_rots = [rest_local_rots[0]]
        parents = [None, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21]
        for i in range(1, 22):
            rest_global_rots.append(rest_global_rots[parents[i]] @ rest_local_rots[i])

        for frame in frames:
            src_loc_rots = []
            for i in range(22):
                src_loc_rots.append(tuple2quat(frame.rots[i]))

            src_global_rots = [src_loc_rots[0]]
            for i in range(1, 22):
                src_global_rots.append(src_global_rots[parents[i]] @ src_loc_rots[i])

            tar_glb_rots = src_global_rots
            for i in range(22):
                tar_glb_rots[i] = tar_glb_rots[i] @ rest_global_rots[i]

            new_loc_rots = [tar_glb_rots[0]]
            for i in range(1, 22):
                new_loc_rots.append(tar_glb_rots[parents[i]].inverted() @ tar_glb_rots[i])

            for i in range(22):
                new_rot = new_loc_rots[i]
                new_rot = new_rot.to_euler()
                frame.rots[i] = (new_rot.x * 180 / math.pi, new_rot.y * 180 / math.pi, new_rot.z * 180 / math.pi)

        # Apply animation
        global_settings = fbx_scene.GetGlobalSettings()
        global_settings.SetTimeMode(fbx.FbxTime.EMode.eFrames60)

        # Create animation stack and layer
        anim_stack = fbx.FbxAnimStack.Create(fbx_scene, "Base Stack")
        anim_layer = fbx.FbxAnimLayer.Create(fbx_scene, "Base Layer")
        anim_stack.AddMember(anim_layer)

        rots = [f.rots for f in frames]
        trans = [[[f.trans[_i] + trans_offset[_i] for _i in range(3)]] for f in frames]
        # Add rotation curves for each joint
        for i in range(22):
            fbx_node = fbx_root_node.FindChild(self.joints[i])
            if fbx_node:
                self.set_curve_value(fbx_node.LclRotation.GetCurve(anim_layer, "X", True), rots,
                                     i, 0)
                self.set_curve_value(fbx_node.LclRotation.GetCurve(anim_layer, "Y", True), rots,
                                     i, 1)
                self.set_curve_value(fbx_node.LclRotation.GetCurve(anim_layer, "Z", True), rots,
                                     i, 2)

        # Add translation curve for "m_avg_Pelvis"
        fbx_node_trans = fbx_root_node.FindChild(self.joints[0])
        if fbx_node_trans:
            self.set_curve_value(fbx_node_trans.LclTranslation.GetCurve(anim_layer, "X", True),
                                 trans, 0, 0)
            self.set_curve_value(fbx_node_trans.LclTranslation.GetCurve(anim_layer, "Y", True),
                                 trans, 0, 1)
            self.set_curve_value(fbx_node_trans.LclTranslation.GetCurve(anim_layer, "Z", True),
                                 trans, 0, 2)

    def set_curve_value(self, curve, datas: List[List[Tuple[float, float, float]]], joint_num, coord_num):
        curve.KeyModifyBegin()
        for i in range(self.total_frame):
            key_time = fbx.FbxTime()
            key_time.SetFrame(i, fbx.FbxTime.EMode.eFrames60)
            key_index = curve.KeyAdd(key_time)[0]
            value = datas[i][joint_num][coord_num]
            curve.KeySetValue(key_index, value)
            curve.KeySetInterpolation(key_index, fbx.FbxAnimCurveDef.EInterpolationType.eInterpolationCubic)
        curve.KeyModifyEnd()
