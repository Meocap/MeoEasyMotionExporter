
import struct
import os
import sys
addon_libs_path = os.path.join(os.path.dirname(__file__), "libs")
if addon_libs_path not in sys.path:
    sys.path.append(addon_libs_path)
from .universal_pb2 import Frame


class BinaryReader:
    def __init__(self, data: bytes):
        self.data = data
        self.position = 0

    def read_uint32(self) -> int:
        length = self.data[self.position:self.position + 4]
        self.position += 4
        return struct.unpack('>I', length)[0]

    def bytes(self) -> bytes:
        length = self.read_uint32()
        value = self.data[self.position:self.position + length]
        self.position += length
        return value

    def eof(self):
        return self.position >= len(self.data)


TRACKER_ATTRS = ["left_lower_arm", "right_lower_arm", "left_lower_leg", "right_lower_leg", "head", "hips",
                 "left_upper_arm", "right_upper_arm", "left_upper_leg", "right_upper_leg", "chest"]


class FrameReader:

    def __init__(self):
        self.frames = []

    def decode_recordings(self, file_path):
        self.frames = []
        with open(file_path, "rb") as f:
            data = f.read()

        reader = BinaryReader(data)

        while not reader.eof():
            frame_data = reader.bytes()
            frame = Frame()
            frame.ParseFromString(frame_data)
            self.frames.append(frame)

        return self.frames
