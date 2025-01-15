import math
import socket
import json
import threading
from dataclasses import dataclass, field
from typing import List, Optional
import mathutils  # Blender's built-in math library


@dataclass
class AnyClass:
    pass


@dataclass
class UniversalFrame:
    frame_id: int
    raw_pose: List[float]
    optimized_pose: List[float]
    glb_opt_pose: List[float]
    translation: List[float]
    joint_positions: List[float]
    joint_velocity: List[float]
    contact: List[float]
    extra_result: Optional[List[AnyClass]]


@dataclass
class SkelJoint:
    pos: List[float]  # 3 elements


@dataclass
class SkelBase:
    bones: List[SkelJoint]  # 24 elements
    floor_y: float


@dataclass
class Joint:
    pos: List[float]  # 3 elements
    glb_rot: mathutils.Quaternion  # 4 elements (quaternion)
    loc_rot: mathutils.Quaternion  # 4 elements (quaternion)


@dataclass
class Addr:
    a: int
    b: int
    c: int
    d: int
    port: int

    def to_socket_addr(self):
        return (f'{self.a}.{self.b}.{self.c}.{self.d}', self.port)

    @classmethod
    def from_socket_addr(cls, socket_addr):
        ip, port = socket_addr
        a, b, c, d = map(int, ip.split('.'))
        return cls(a, b, c, d, port)


@dataclass
class MeoFrame:
    frame_id: int
    translation: mathutils.Vector  # 3 elements
    joints: List[Joint]  # 24 elements
    src: Addr


class Status:
    def __init__(self, ty, info):
        self.ty = ty
        self.info = info


class ErrorType:
    NONE = 0
    SOCKET = 1
    INVALID_PARAMETER = 2
    DATA_CORRUPTED = 3


class MeocapSDK:
    def __init__(self, addr: Addr):
        self.socket = None
        self.addr = addr
        self.stop_event: Optional[threading.Event] = None
        self.frame: Optional[MeoFrame] = None

    def bind(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(self.addr.to_socket_addr())
            self.socket.settimeout(0.25)
        except socket.error as e:
            return Status(ErrorType.SOCKET, e.errno)
        self.stop_event = threading.Event()
        t = threading.Thread(target=self.recv_thread)
        t.start()
        return Status(ErrorType.NONE, 0)

    def send_skel(self, skel: SkelBase):
        try:
            data = json.dumps({"SetSkel": skel}).encode('utf-8')
            self.socket.sendto(data, self.addr.to_socket_addr())
        except Exception as e:
            return Status(ErrorType.SOCKET, str(e))
        return Status(ErrorType.NONE, 0)

    def get_last_frame(self):
        return self.frame

    def recv_frame(self) -> Optional[MeoFrame]:
        try:
            data, src = self.socket.recvfrom(65536)
            frame_json = json.loads(data.decode('utf-8'))
            frame = UniversalFrame(**frame_json)

            meo_frame = MeoFrame(
                frame_id=frame.frame_id,
                translation=mathutils.Vector((frame.translation[0], -frame.translation[2], frame.translation[1])),
                joints=[Joint([0, 0, 0], None, None) for _ in range(24)],
                src=Addr.from_socket_addr(src)
            )

            def convert_axis(q: mathutils.Quaternion):
                return mathutils.Quaternion(mathutils.Vector([q.w, q.x, -q.z, q.y]))

            for i in range(24):
                rot = frame.glb_opt_pose[i * 9:(i + 1) * 9]
                pos = frame.joint_positions[i * 3:(i + 1) * 3]
                rot_matrix = mathutils.Matrix()
                rot_matrix[0][0:3] = rot[0], rot[1], rot[2]
                rot_matrix[1][0:3] = rot[3], rot[4], rot[5]
                rot_matrix[2][0:3] = rot[6], rot[7], rot[8]
                r_glb = rot_matrix.to_quaternion().inverted()
                meo_frame.joints[i].pos = pos
                meo_frame.joints[i].glb_rot = convert_axis(r_glb)

                rot = frame.optimized_pose[i * 9:(i + 1) * 9]
                rot_matrix = mathutils.Matrix()
                rot_matrix[0][0:3] = rot[0], rot[1], rot[2]
                rot_matrix[1][0:3] = rot[3], rot[4], rot[5]
                rot_matrix[2][0:3] = rot[6], rot[7], rot[8]
                r_loc = rot_matrix.to_quaternion().inverted()
                meo_frame.joints[i].loc_rot = convert_axis(r_loc)
            '''
            for i in range(24):
                meo_frame.joints[i].loc_rot = mathutils.Matrix.Identity(4).to_quaternion()
                if i == 16:
                    rotation_matrix = mathutils.Matrix.Rotation(math.radians(90), 4, 'Y')
                    meo_frame.joints[i].loc_rot = rotation_matrix.to_quaternion()
            '''

            return meo_frame
        except Exception as e:
            return None

    def close(self):
        if self.stop_event is not None:
            self.stop_event.set()
            self.stop_event = None
        if self.socket:
            self.socket.close()
            self.socket = None
        return Status(ErrorType.NONE, 0)

    def recv_thread(self):
        while (self.stop_event is not None) and (not self.stop_event.is_set()):
            self.frame = self.recv_frame()


# Example usage in Blender:
if __name__ == "__main__":
    addr = Addr(127, 0, 0, 1, 14999)
    connection = MeocapSDK(addr)
    status = connection.bind()
    while True:
        if status.ty == ErrorType.NONE:
            frame = connection.recv_frame()
            if frame:
                print(f"Received frame {frame.frame_id}")
    connection.close()
