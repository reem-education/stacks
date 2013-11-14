"""autogenerated by genpy from blort_ros/TrackerConfidences.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class TrackerConfidences(genpy.Message):
  _md5sum = "1efbea2b74f43081141e5d758958c1c5"
  _type = "blort_ros/TrackerConfidences"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 edgeConf
float32 confThreshold
float32 lostConf
float32 distance

"""
  __slots__ = ['edgeConf','confThreshold','lostConf','distance']
  _slot_types = ['float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       edgeConf,confThreshold,lostConf,distance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TrackerConfidences, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.edgeConf is None:
        self.edgeConf = 0.
      if self.confThreshold is None:
        self.confThreshold = 0.
      if self.lostConf is None:
        self.lostConf = 0.
      if self.distance is None:
        self.distance = 0.
    else:
      self.edgeConf = 0.
      self.confThreshold = 0.
      self.lostConf = 0.
      self.distance = 0.

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
      buff.write(_struct_4f.pack(_x.edgeConf, _x.confThreshold, _x.lostConf, _x.distance))
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
      (_x.edgeConf, _x.confThreshold, _x.lostConf, _x.distance,) = _struct_4f.unpack(str[start:end])
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
      buff.write(_struct_4f.pack(_x.edgeConf, _x.confThreshold, _x.lostConf, _x.distance))
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
      (_x.edgeConf, _x.confThreshold, _x.lostConf, _x.distance,) = _struct_4f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4f = struct.Struct("<4f")
