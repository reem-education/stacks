"""autogenerated by genpy from control_msgs/GripperCommand.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class GripperCommand(genpy.Message):
  _md5sum = "680acaff79486f017132a7f198d40f08"
  _type = "control_msgs/GripperCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 position
float64 max_effort

"""
  __slots__ = ['position','max_effort']
  _slot_types = ['float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       position,max_effort

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(GripperCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.position is None:
        self.position = 0.
      if self.max_effort is None:
        self.max_effort = 0.
    else:
      self.position = 0.
      self.max_effort = 0.

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
      buff.write(_struct_2d.pack(_x.position, _x.max_effort))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.position, _x.max_effort,) = _struct_2d.unpack(str[start:end])
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
      buff.write(_struct_2d.pack(_x.position, _x.max_effort))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.position, _x.max_effort,) = _struct_2d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
