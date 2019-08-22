class JSONSerializable(object):
    # Implement in your subclasses if you need a custom factory
    # @classmethod
    # def json_factory(cls, **kwargs):
    #     raise NotImplementedError

    def json_data(self):
        out = {'__type__': str(type(self))}
        self._json_data(out)
        return out

    def _json_data(self, json_dict):
        raise NotImplementedError