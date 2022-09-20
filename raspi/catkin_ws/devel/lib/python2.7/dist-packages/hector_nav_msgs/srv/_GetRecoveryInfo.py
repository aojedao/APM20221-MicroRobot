# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hector_nav_msgs/GetRecoveryInfoRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class GetRecoveryInfoRequest(genpy.Message):
  _md5sum = "3916a0c55958d5dd43204cd2fe5608f6"
  _type = "hector_nav_msgs/GetRecoveryInfoRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """



time request_time
float64 request_radius
"""
  __slots__ = ['request_time','request_radius']
  _slot_types = ['time','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       request_time,request_radius

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetRecoveryInfoRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.request_time is None:
        self.request_time = genpy.Time()
      if self.request_radius is None:
        self.request_radius = 0.
    else:
      self.request_time = genpy.Time()
      self.request_radius = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_2Id().pack(_x.request_time.secs, _x.request_time.nsecs, _x.request_radius))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.request_time is None:
        self.request_time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.request_time.secs, _x.request_time.nsecs, _x.request_radius,) = _get_struct_2Id().unpack(str[start:end])
      self.request_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2Id().pack(_x.request_time.secs, _x.request_time.nsecs, _x.request_radius))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.request_time is None:
        self.request_time = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.request_time.secs, _x.request_time.nsecs, _x.request_radius,) = _get_struct_2Id().unpack(str[start:end])
      self.request_time.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2Id = None
def _get_struct_2Id():
    global _struct_2Id
    if _struct_2Id is None:
        _struct_2Id = struct.Struct("<2Id")
    return _struct_2Id
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from hector_nav_msgs/GetRecoveryInfoResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

class GetRecoveryInfoResponse(genpy.Message):
  _md5sum = "a93581be8e34e3c09aeafc6b9b990ad5"
  _type = "hector_nav_msgs/GetRecoveryInfoResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """nav_msgs/Path trajectory_radius_entry_pose_to_req_pose
geometry_msgs/PoseStamped radius_entry_pose
geometry_msgs/PoseStamped req_pose



================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['trajectory_radius_entry_pose_to_req_pose','radius_entry_pose','req_pose']
  _slot_types = ['nav_msgs/Path','geometry_msgs/PoseStamped','geometry_msgs/PoseStamped']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       trajectory_radius_entry_pose_to_req_pose,radius_entry_pose,req_pose

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GetRecoveryInfoResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.trajectory_radius_entry_pose_to_req_pose is None:
        self.trajectory_radius_entry_pose_to_req_pose = nav_msgs.msg.Path()
      if self.radius_entry_pose is None:
        self.radius_entry_pose = geometry_msgs.msg.PoseStamped()
      if self.req_pose is None:
        self.req_pose = geometry_msgs.msg.PoseStamped()
    else:
      self.trajectory_radius_entry_pose_to_req_pose = nav_msgs.msg.Path()
      self.radius_entry_pose = geometry_msgs.msg.PoseStamped()
      self.req_pose = geometry_msgs.msg.PoseStamped()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.trajectory_radius_entry_pose_to_req_pose.header.seq, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.secs, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.nsecs))
      _x = self.trajectory_radius_entry_pose_to_req_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.trajectory_radius_entry_pose_to_req_pose.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.trajectory_radius_entry_pose_to_req_pose.poses:
        _v1 = val1.header
        buff.write(_get_struct_I().pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v3 = val1.pose
        _v4 = _v3.position
        _x = _v4
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v5 = _v3.orientation
        _x = _v5
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_get_struct_3I().pack(_x.radius_entry_pose.header.seq, _x.radius_entry_pose.header.stamp.secs, _x.radius_entry_pose.header.stamp.nsecs))
      _x = self.radius_entry_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7d3I().pack(_x.radius_entry_pose.pose.position.x, _x.radius_entry_pose.pose.position.y, _x.radius_entry_pose.pose.position.z, _x.radius_entry_pose.pose.orientation.x, _x.radius_entry_pose.pose.orientation.y, _x.radius_entry_pose.pose.orientation.z, _x.radius_entry_pose.pose.orientation.w, _x.req_pose.header.seq, _x.req_pose.header.stamp.secs, _x.req_pose.header.stamp.nsecs))
      _x = self.req_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.req_pose.pose.position.x, _x.req_pose.pose.position.y, _x.req_pose.pose.position.z, _x.req_pose.pose.orientation.x, _x.req_pose.pose.orientation.y, _x.req_pose.pose.orientation.z, _x.req_pose.pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.trajectory_radius_entry_pose_to_req_pose is None:
        self.trajectory_radius_entry_pose_to_req_pose = nav_msgs.msg.Path()
      if self.radius_entry_pose is None:
        self.radius_entry_pose = geometry_msgs.msg.PoseStamped()
      if self.req_pose is None:
        self.req_pose = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.trajectory_radius_entry_pose_to_req_pose.header.seq, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.secs, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.trajectory_radius_entry_pose_to_req_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.trajectory_radius_entry_pose_to_req_pose.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.trajectory_radius_entry_pose_to_req_pose.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.PoseStamped()
        _v6 = val1.header
        start = end
        end += 4
        (_v6.seq,) = _get_struct_I().unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v6.frame_id = str[start:end].decode('utf-8')
        else:
          _v6.frame_id = str[start:end]
        _v8 = val1.pose
        _v9 = _v8.position
        _x = _v9
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v10 = _v8.orientation
        _x = _v10
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.trajectory_radius_entry_pose_to_req_pose.poses.append(val1)
      _x = self
      start = end
      end += 12
      (_x.radius_entry_pose.header.seq, _x.radius_entry_pose.header.stamp.secs, _x.radius_entry_pose.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.radius_entry_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.radius_entry_pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 68
      (_x.radius_entry_pose.pose.position.x, _x.radius_entry_pose.pose.position.y, _x.radius_entry_pose.pose.position.z, _x.radius_entry_pose.pose.orientation.x, _x.radius_entry_pose.pose.orientation.y, _x.radius_entry_pose.pose.orientation.z, _x.radius_entry_pose.pose.orientation.w, _x.req_pose.header.seq, _x.req_pose.header.stamp.secs, _x.req_pose.header.stamp.nsecs,) = _get_struct_7d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.req_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.req_pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.req_pose.pose.position.x, _x.req_pose.pose.position.y, _x.req_pose.pose.position.z, _x.req_pose.pose.orientation.x, _x.req_pose.pose.orientation.y, _x.req_pose.pose.orientation.z, _x.req_pose.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.trajectory_radius_entry_pose_to_req_pose.header.seq, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.secs, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.nsecs))
      _x = self.trajectory_radius_entry_pose_to_req_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.trajectory_radius_entry_pose_to_req_pose.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.trajectory_radius_entry_pose_to_req_pose.poses:
        _v11 = val1.header
        buff.write(_get_struct_I().pack(_v11.seq))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v13 = val1.pose
        _v14 = _v13.position
        _x = _v14
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v15 = _v13.orientation
        _x = _v15
        buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
      _x = self
      buff.write(_get_struct_3I().pack(_x.radius_entry_pose.header.seq, _x.radius_entry_pose.header.stamp.secs, _x.radius_entry_pose.header.stamp.nsecs))
      _x = self.radius_entry_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7d3I().pack(_x.radius_entry_pose.pose.position.x, _x.radius_entry_pose.pose.position.y, _x.radius_entry_pose.pose.position.z, _x.radius_entry_pose.pose.orientation.x, _x.radius_entry_pose.pose.orientation.y, _x.radius_entry_pose.pose.orientation.z, _x.radius_entry_pose.pose.orientation.w, _x.req_pose.header.seq, _x.req_pose.header.stamp.secs, _x.req_pose.header.stamp.nsecs))
      _x = self.req_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.req_pose.pose.position.x, _x.req_pose.pose.position.y, _x.req_pose.pose.position.z, _x.req_pose.pose.orientation.x, _x.req_pose.pose.orientation.y, _x.req_pose.pose.orientation.z, _x.req_pose.pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.trajectory_radius_entry_pose_to_req_pose is None:
        self.trajectory_radius_entry_pose_to_req_pose = nav_msgs.msg.Path()
      if self.radius_entry_pose is None:
        self.radius_entry_pose = geometry_msgs.msg.PoseStamped()
      if self.req_pose is None:
        self.req_pose = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.trajectory_radius_entry_pose_to_req_pose.header.seq, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.secs, _x.trajectory_radius_entry_pose_to_req_pose.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.trajectory_radius_entry_pose_to_req_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.trajectory_radius_entry_pose_to_req_pose.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.trajectory_radius_entry_pose_to_req_pose.poses = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.PoseStamped()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _get_struct_I().unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v16.frame_id = str[start:end].decode('utf-8')
        else:
          _v16.frame_id = str[start:end]
        _v18 = val1.pose
        _v19 = _v18.position
        _x = _v19
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v20 = _v18.orientation
        _x = _v20
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
        self.trajectory_radius_entry_pose_to_req_pose.poses.append(val1)
      _x = self
      start = end
      end += 12
      (_x.radius_entry_pose.header.seq, _x.radius_entry_pose.header.stamp.secs, _x.radius_entry_pose.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.radius_entry_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.radius_entry_pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 68
      (_x.radius_entry_pose.pose.position.x, _x.radius_entry_pose.pose.position.y, _x.radius_entry_pose.pose.position.z, _x.radius_entry_pose.pose.orientation.x, _x.radius_entry_pose.pose.orientation.y, _x.radius_entry_pose.pose.orientation.z, _x.radius_entry_pose.pose.orientation.w, _x.req_pose.header.seq, _x.req_pose.header.stamp.secs, _x.req_pose.header.stamp.nsecs,) = _get_struct_7d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.req_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.req_pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.req_pose.pose.position.x, _x.req_pose.pose.position.y, _x.req_pose.pose.position.z, _x.req_pose.pose.orientation.x, _x.req_pose.pose.orientation.y, _x.req_pose.pose.orientation.z, _x.req_pose.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_7d3I = None
def _get_struct_7d3I():
    global _struct_7d3I
    if _struct_7d3I is None:
        _struct_7d3I = struct.Struct("<7d3I")
    return _struct_7d3I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
class GetRecoveryInfo(object):
  _type          = 'hector_nav_msgs/GetRecoveryInfo'
  _md5sum = 'edd6e579a08e5c27f2b7fcfa4c39b7bb'
  _request_class  = GetRecoveryInfoRequest
  _response_class = GetRecoveryInfoResponse
