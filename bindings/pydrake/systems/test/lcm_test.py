import pydrake.systems.lcm as mut

import unittest

from robotlocomotion import quaternion_t

from pydrake.lcm import DrakeMockLcm
from pydrake.systems.framework import AbstractValue

# Create a simple template message.
MSG = quaternion_t()
MSG.w, MSG.x, MSG.y, MSG.z = (1, 2, 3, 4)


class TestSystemsLcm(unittest.TestCase):
    def test_serializer(self):
        dut = mut.PySerializer(quaternion_t)
        value = dut.CreateDefaultValue()
        self.assertIsInstance(value.get_value(), quaternion_t)
        self.assertNotEqual(value.get_value().encode(), MSG.encode())
        dut.Deserialize(MSG.encode(), value)
        self.assertEqual(value.get_value().encode(), MSG.encode())
        raw = dut.Serialize(value)
        self.assertEqual(raw, MSG.encode())

    def test_subscriber(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmSubscriberSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        lcm.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=MSG.encode())
        context = dut.CreateDefaultContext()
        output = dut.AllocateOutput(context)
        dut.CalcOutput(context, output)
        actual = output.get_data(0).get_value()
        self.assertEqual(actual.encode(), MSG.encode())

    def test_publisher(self):
        lcm = DrakeMockLcm()
        dut = mut.LcmPublisherSystem.Make(
            channel="TEST_CHANNEL", lcm_type=quaternion_t, lcm=lcm)
        dut.set_publish_period(period=0.1)
        context = dut.CreateDefaultContext()
        context.FixInputPort(0, AbstractValue.Make(MSG))
        dut.Publish(context)
        raw = lcm.get_last_published_message("TEST_CHANNEL")
        self.assertEqual(raw, MSG.encode())
