
class MessageBase:
    """MessageBase class.

    All message types will be derived from this.
    """
    _tov = None

    def __init__(self, tov):
        #print('Init MessageBase')
        self._tov = tov