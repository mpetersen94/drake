from pydrake.systems._lcm_py import *

from pydrake.systems.framework import AbstractValue, Value


class PySerializer(SerializerInterface):
    def __init__(self, lcm_type):
        SerializerInterface.__init__(self)
        self._lcm_type = lcm_type

    def CreateDefaultValue(self):
        return AbstractValue.Make(self._lcm_type())

    def Deserialize(self, buffer, value):
        msg = self._lcm_type.decode(buffer)
        value.set_value(msg)

    def Serialize(self, value):
        assert isinstance(value, AbstractValue)
        msg = value.get_value()
        assert isinstance(msg, self._lcm_type)
        return msg.encode()


# Attach methods (as descriptors).
@classmethod
def _make_lcm_system(cls, channel, lcm_type, lcm):
    serializer = PySerializer(lcm_type)
    return cls(channel, serializer, lcm)


LcmPublisherSystem.Make = _make_lcm_system
LcmSubscriberSystem.Make = _make_lcm_system
