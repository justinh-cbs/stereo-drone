from shared_buffer import StreamBase, MessageBase

class SetpointMessage(MessageBase):
    """Setpoint Message class.

    """

    def __init__(self, tov, roll, pitch, yaw, thrust):
        #print('SetpointMessage')
        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw
        self._thrust = thrust
        super(SetpointMessage, self).__init__(tov)


class SetpointStream(StreamBase):
    """Setpoint Stream class.

    """
    _setpoint_buffer_length = 10

    def __init__(self):
        print('SetpointStream')
        super(SetpointStream, self).__init__(self._setpoint_buffer_length)