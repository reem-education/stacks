"""autogenerated by genpy from atlas_msgs/AtlasState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import osrf_msgs.msg
import atlas_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

class AtlasState(genpy.Message):
  _md5sum = "acce2e16a35dfc26fe7c270017a83ffb"
  _type = "atlas_msgs/AtlasState"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# testing everything a robot needs
Header header

# Default joint indices used when publishing the
# JointCommands joint_states topic below
# For exmaple, if you subscribe to this message, then
# msg.joint_states.position[atlas_msgs::AtlasStates::back_lbz] gives back
# the position of the back_lbz.
int32 back_lbz  = 0
int32 back_mby  = 1
int32 back_ubx  = 2
int32 neck_ay   = 3
int32 l_leg_uhz = 4
int32 l_leg_mhx = 5
int32 l_leg_lhy = 6
int32 l_leg_kny = 7
int32 l_leg_uay = 8
int32 l_leg_lax = 9
int32 r_leg_uhz = 10
int32 r_leg_mhx = 11
int32 r_leg_lhy = 12
int32 r_leg_kny = 13
int32 r_leg_uay = 14
int32 r_leg_lax = 15
int32 l_arm_usy = 16
int32 l_arm_shx = 17
int32 l_arm_ely = 18
int32 l_arm_elx = 19
int32 l_arm_uwy = 20
int32 l_arm_mwx = 21
int32 r_arm_usy = 22
int32 r_arm_shx = 23
int32 r_arm_ely = 24
int32 r_arm_elx = 25
int32 r_arm_uwy = 26
int32 r_arm_mwx = 27

# using JointCommands to store joint_states information
# joint_states.name[] array is left empty, use the enum list above
osrf_msgs/JointCommands joint_states
sensor_msgs/Imu imu
atlas_msgs/ForceTorqueSensors force_torque_sensors

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: osrf_msgs/JointCommands
# Joint Command Message
# This structure contains the gains to be applied to a joint.
# The controller is a PID with feedforward desired torque:
#
#   kp_position     * ( position - measured_position )       +
#   ki_position     * 1/s * ( position - measured_position ) +
#   kd_position     * s * ( position - measured_position ) +
#   kp_velocity    * ( velocity - measured_velocity )     +
#   effort
#
Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

float64[] kp_position
float64[] ki_position
float64[] kd_position
float64[] kp_velocity

float64[] i_effort_min
float64[] i_effort_max

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: atlas_msgs/ForceTorqueSensors
# Atlas force torque sensors for the wrists and ankles
Header header

geometry_msgs/Wrench l_foot
geometry_msgs/Wrench r_foot
geometry_msgs/Wrench l_hand
geometry_msgs/Wrench r_hand

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, seperated into 
# it's linear and angular parts.  
Vector3  force
Vector3  torque

"""
  # Pseudo-constants
  back_lbz = 0
  back_mby = 1
  back_ubx = 2
  neck_ay = 3
  l_leg_uhz = 4
  l_leg_mhx = 5
  l_leg_lhy = 6
  l_leg_kny = 7
  l_leg_uay = 8
  l_leg_lax = 9
  r_leg_uhz = 10
  r_leg_mhx = 11
  r_leg_lhy = 12
  r_leg_kny = 13
  r_leg_uay = 14
  r_leg_lax = 15
  l_arm_usy = 16
  l_arm_shx = 17
  l_arm_ely = 18
  l_arm_elx = 19
  l_arm_uwy = 20
  l_arm_mwx = 21
  r_arm_usy = 22
  r_arm_shx = 23
  r_arm_ely = 24
  r_arm_elx = 25
  r_arm_uwy = 26
  r_arm_mwx = 27

  __slots__ = ['header','joint_states','imu','force_torque_sensors']
  _slot_types = ['std_msgs/Header','osrf_msgs/JointCommands','sensor_msgs/Imu','atlas_msgs/ForceTorqueSensors']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,joint_states,imu,force_torque_sensors

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AtlasState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.joint_states is None:
        self.joint_states = osrf_msgs.msg.JointCommands()
      if self.imu is None:
        self.imu = sensor_msgs.msg.Imu()
      if self.force_torque_sensors is None:
        self.force_torque_sensors = atlas_msgs.msg.ForceTorqueSensors()
    else:
      self.header = std_msgs.msg.Header()
      self.joint_states = osrf_msgs.msg.JointCommands()
      self.imu = sensor_msgs.msg.Imu()
      self.force_torque_sensors = atlas_msgs.msg.ForceTorqueSensors()

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.joint_states.header.seq, _x.joint_states.header.stamp.secs, _x.joint_states.header.stamp.nsecs))
      _x = self.joint_states.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.joint_states.name)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_states.name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.joint_states.position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.position))
      length = len(self.joint_states.velocity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.velocity))
      length = len(self.joint_states.effort)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.effort))
      length = len(self.joint_states.kp_position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.kp_position))
      length = len(self.joint_states.ki_position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.ki_position))
      length = len(self.joint_states.kd_position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.kd_position))
      length = len(self.joint_states.kp_velocity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.kp_velocity))
      length = len(self.joint_states.i_effort_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.i_effort_min))
      length = len(self.joint_states.i_effort_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.joint_states.i_effort_max))
      _x = self
      buff.write(_struct_3I.pack(_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs))
      _x = self.imu.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_4d.pack(_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w))
      buff.write(_struct_9d.pack(*self.imu.orientation_covariance))
      _x = self
      buff.write(_struct_3d.pack(_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z))
      buff.write(_struct_9d.pack(*self.imu.angular_velocity_covariance))
      _x = self
      buff.write(_struct_3d.pack(_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z))
      buff.write(_struct_9d.pack(*self.imu.linear_acceleration_covariance))
      _x = self
      buff.write(_struct_3I.pack(_x.force_torque_sensors.header.seq, _x.force_torque_sensors.header.stamp.secs, _x.force_torque_sensors.header.stamp.nsecs))
      _x = self.force_torque_sensors.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_24d.pack(_x.force_torque_sensors.l_foot.force.x, _x.force_torque_sensors.l_foot.force.y, _x.force_torque_sensors.l_foot.force.z, _x.force_torque_sensors.l_foot.torque.x, _x.force_torque_sensors.l_foot.torque.y, _x.force_torque_sensors.l_foot.torque.z, _x.force_torque_sensors.r_foot.force.x, _x.force_torque_sensors.r_foot.force.y, _x.force_torque_sensors.r_foot.force.z, _x.force_torque_sensors.r_foot.torque.x, _x.force_torque_sensors.r_foot.torque.y, _x.force_torque_sensors.r_foot.torque.z, _x.force_torque_sensors.l_hand.force.x, _x.force_torque_sensors.l_hand.force.y, _x.force_torque_sensors.l_hand.force.z, _x.force_torque_sensors.l_hand.torque.x, _x.force_torque_sensors.l_hand.torque.y, _x.force_torque_sensors.l_hand.torque.z, _x.force_torque_sensors.r_hand.force.x, _x.force_torque_sensors.r_hand.force.y, _x.force_torque_sensors.r_hand.force.z, _x.force_torque_sensors.r_hand.torque.x, _x.force_torque_sensors.r_hand.torque.y, _x.force_torque_sensors.r_hand.torque.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.joint_states is None:
        self.joint_states = osrf_msgs.msg.JointCommands()
      if self.imu is None:
        self.imu = sensor_msgs.msg.Imu()
      if self.force_torque_sensors is None:
        self.force_torque_sensors = atlas_msgs.msg.ForceTorqueSensors()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.joint_states.header.seq, _x.joint_states.header.stamp.secs, _x.joint_states.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.joint_states.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.joint_states.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_states.name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.joint_states.name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.position = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.velocity = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.effort = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.kp_position = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.ki_position = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.kd_position = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.kp_velocity = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.i_effort_min = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.i_effort_max = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 12
      (_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.imu.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.imu.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 32
      (_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w,) = _struct_4d.unpack(str[start:end])
      start = end
      end += 72
      self.imu.orientation_covariance = _struct_9d.unpack(str[start:end])
      _x = self
      start = end
      end += 24
      (_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z,) = _struct_3d.unpack(str[start:end])
      start = end
      end += 72
      self.imu.angular_velocity_covariance = _struct_9d.unpack(str[start:end])
      _x = self
      start = end
      end += 24
      (_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z,) = _struct_3d.unpack(str[start:end])
      start = end
      end += 72
      self.imu.linear_acceleration_covariance = _struct_9d.unpack(str[start:end])
      _x = self
      start = end
      end += 12
      (_x.force_torque_sensors.header.seq, _x.force_torque_sensors.header.stamp.secs, _x.force_torque_sensors.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.force_torque_sensors.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.force_torque_sensors.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 192
      (_x.force_torque_sensors.l_foot.force.x, _x.force_torque_sensors.l_foot.force.y, _x.force_torque_sensors.l_foot.force.z, _x.force_torque_sensors.l_foot.torque.x, _x.force_torque_sensors.l_foot.torque.y, _x.force_torque_sensors.l_foot.torque.z, _x.force_torque_sensors.r_foot.force.x, _x.force_torque_sensors.r_foot.force.y, _x.force_torque_sensors.r_foot.force.z, _x.force_torque_sensors.r_foot.torque.x, _x.force_torque_sensors.r_foot.torque.y, _x.force_torque_sensors.r_foot.torque.z, _x.force_torque_sensors.l_hand.force.x, _x.force_torque_sensors.l_hand.force.y, _x.force_torque_sensors.l_hand.force.z, _x.force_torque_sensors.l_hand.torque.x, _x.force_torque_sensors.l_hand.torque.y, _x.force_torque_sensors.l_hand.torque.z, _x.force_torque_sensors.r_hand.force.x, _x.force_torque_sensors.r_hand.force.y, _x.force_torque_sensors.r_hand.force.z, _x.force_torque_sensors.r_hand.torque.x, _x.force_torque_sensors.r_hand.torque.y, _x.force_torque_sensors.r_hand.torque.z,) = _struct_24d.unpack(str[start:end])
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.joint_states.header.seq, _x.joint_states.header.stamp.secs, _x.joint_states.header.stamp.nsecs))
      _x = self.joint_states.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.joint_states.name)
      buff.write(_struct_I.pack(length))
      for val1 in self.joint_states.name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.joint_states.position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.position.tostring())
      length = len(self.joint_states.velocity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.velocity.tostring())
      length = len(self.joint_states.effort)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.effort.tostring())
      length = len(self.joint_states.kp_position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.kp_position.tostring())
      length = len(self.joint_states.ki_position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.ki_position.tostring())
      length = len(self.joint_states.kd_position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.kd_position.tostring())
      length = len(self.joint_states.kp_velocity)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.kp_velocity.tostring())
      length = len(self.joint_states.i_effort_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.i_effort_min.tostring())
      length = len(self.joint_states.i_effort_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.joint_states.i_effort_max.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs))
      _x = self.imu.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_4d.pack(_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w))
      buff.write(self.imu.orientation_covariance.tostring())
      _x = self
      buff.write(_struct_3d.pack(_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z))
      buff.write(self.imu.angular_velocity_covariance.tostring())
      _x = self
      buff.write(_struct_3d.pack(_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z))
      buff.write(self.imu.linear_acceleration_covariance.tostring())
      _x = self
      buff.write(_struct_3I.pack(_x.force_torque_sensors.header.seq, _x.force_torque_sensors.header.stamp.secs, _x.force_torque_sensors.header.stamp.nsecs))
      _x = self.force_torque_sensors.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_24d.pack(_x.force_torque_sensors.l_foot.force.x, _x.force_torque_sensors.l_foot.force.y, _x.force_torque_sensors.l_foot.force.z, _x.force_torque_sensors.l_foot.torque.x, _x.force_torque_sensors.l_foot.torque.y, _x.force_torque_sensors.l_foot.torque.z, _x.force_torque_sensors.r_foot.force.x, _x.force_torque_sensors.r_foot.force.y, _x.force_torque_sensors.r_foot.force.z, _x.force_torque_sensors.r_foot.torque.x, _x.force_torque_sensors.r_foot.torque.y, _x.force_torque_sensors.r_foot.torque.z, _x.force_torque_sensors.l_hand.force.x, _x.force_torque_sensors.l_hand.force.y, _x.force_torque_sensors.l_hand.force.z, _x.force_torque_sensors.l_hand.torque.x, _x.force_torque_sensors.l_hand.torque.y, _x.force_torque_sensors.l_hand.torque.z, _x.force_torque_sensors.r_hand.force.x, _x.force_torque_sensors.r_hand.force.y, _x.force_torque_sensors.r_hand.force.z, _x.force_torque_sensors.r_hand.torque.x, _x.force_torque_sensors.r_hand.torque.y, _x.force_torque_sensors.r_hand.torque.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.joint_states is None:
        self.joint_states = osrf_msgs.msg.JointCommands()
      if self.imu is None:
        self.imu = sensor_msgs.msg.Imu()
      if self.force_torque_sensors is None:
        self.force_torque_sensors = atlas_msgs.msg.ForceTorqueSensors()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.joint_states.header.seq, _x.joint_states.header.stamp.secs, _x.joint_states.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.joint_states.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.joint_states.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.joint_states.name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.joint_states.name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.position = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.velocity = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.effort = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.kp_position = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.ki_position = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.kd_position = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.kp_velocity = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.i_effort_min = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.joint_states.i_effort_max = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 12
      (_x.imu.header.seq, _x.imu.header.stamp.secs, _x.imu.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.imu.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.imu.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 32
      (_x.imu.orientation.x, _x.imu.orientation.y, _x.imu.orientation.z, _x.imu.orientation.w,) = _struct_4d.unpack(str[start:end])
      start = end
      end += 72
      self.imu.orientation_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      _x = self
      start = end
      end += 24
      (_x.imu.angular_velocity.x, _x.imu.angular_velocity.y, _x.imu.angular_velocity.z,) = _struct_3d.unpack(str[start:end])
      start = end
      end += 72
      self.imu.angular_velocity_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      _x = self
      start = end
      end += 24
      (_x.imu.linear_acceleration.x, _x.imu.linear_acceleration.y, _x.imu.linear_acceleration.z,) = _struct_3d.unpack(str[start:end])
      start = end
      end += 72
      self.imu.linear_acceleration_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=9)
      _x = self
      start = end
      end += 12
      (_x.force_torque_sensors.header.seq, _x.force_torque_sensors.header.stamp.secs, _x.force_torque_sensors.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.force_torque_sensors.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.force_torque_sensors.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 192
      (_x.force_torque_sensors.l_foot.force.x, _x.force_torque_sensors.l_foot.force.y, _x.force_torque_sensors.l_foot.force.z, _x.force_torque_sensors.l_foot.torque.x, _x.force_torque_sensors.l_foot.torque.y, _x.force_torque_sensors.l_foot.torque.z, _x.force_torque_sensors.r_foot.force.x, _x.force_torque_sensors.r_foot.force.y, _x.force_torque_sensors.r_foot.force.z, _x.force_torque_sensors.r_foot.torque.x, _x.force_torque_sensors.r_foot.torque.y, _x.force_torque_sensors.r_foot.torque.z, _x.force_torque_sensors.l_hand.force.x, _x.force_torque_sensors.l_hand.force.y, _x.force_torque_sensors.l_hand.force.z, _x.force_torque_sensors.l_hand.torque.x, _x.force_torque_sensors.l_hand.torque.y, _x.force_torque_sensors.l_hand.torque.z, _x.force_torque_sensors.r_hand.force.x, _x.force_torque_sensors.r_hand.force.y, _x.force_torque_sensors.r_hand.force.z, _x.force_torque_sensors.r_hand.torque.x, _x.force_torque_sensors.r_hand.torque.y, _x.force_torque_sensors.r_hand.torque.z,) = _struct_24d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_24d = struct.Struct("<24d")
_struct_3I = struct.Struct("<3I")
_struct_3d = struct.Struct("<3d")
_struct_4d = struct.Struct("<4d")
_struct_9d = struct.Struct("<9d")