from shared_buffer import StreamBase, MessageBase

class FeatureMessage(MessageBase):
    """Feature Message class.

    """

    def __init__(self, tov, df):
        #print('Feature Message')
        self.feature_df = df
        super(FeatureMessage, self).__init__(tov)


class FeatureStream(StreamBase):
    """Feature Stream class.

    """
    _feature_buffer_length = 10

    def __init__(self):
        #print('Feature Stream')
        super(FeatureStream, self).__init__(self._feature_buffer_length)