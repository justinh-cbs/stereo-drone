from shared_buffer import StreamBase, MessageBase

class IMUMessage(MessageBase):
    """IMU Message class.

    """

    def __init__(self, tov, acceleration, rate):
        #print('IMU Message')
        self._acceleration = acceleration
        self._rate = rate
        super(IMUMessage, self).__init__(tov)


class IMUStream(StreamBase):
    """IMU Stream class.

    """
    _imu_buffer_length = 100

    def __init__(self):
        #print('IMU Stream')
        super(IMUStream, self).__init__(self._imu_buffer_length)