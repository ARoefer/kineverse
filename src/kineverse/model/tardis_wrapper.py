from kineverse.time_wrapper          import Time
from kineverse.model.history         import Timeline, StampedData
from kineverse.model.kinematic_model import KinematicModel


def TARDIS(super_type, *args):
    if not issubclass(super_type, KinematicModel):
        raise Exception('Type must be a subtype of KinematicModel. Given type: {}'.format(super_type))

    def init(self, *args):
        super_type.__init__(self, *args)
        self.operations_timeline = Timeline()
        self.removal_timeline    = Timeline()

    def apply_operation(self, op, tag):
        super_type.apply_operation(self, op, tag)
        self.operations_timeline.add(StampedData(Time.now(), operation=op, tag=tag))

    def apply_operation_before(self, op, tag, before_tag):
        super_type.apply_operation_before(self, op, tag, before_tag)
        self.operations_timeline.add(StampedData(Time.now(), operation=op, tag=tag))    

    def apply_operation_after(self, op, tag, after_tag):
        super_type.apply_operation_after(self, op, tag, after_tag)
        self.operations_timeline.add(StampedData(Time.now(), operation=op, tag=tag))    

    def remove_operation(self, tag):
        super_type.remove_operation(self, tag)
        rem_set = self.removal_timeline[-1] if len(self.removal_timeline) > 0 else set()
        self.removal_timeline.add(StampedData(Time.now(), rem_set.union({tag})))

    def get_model_at_time(self, time):
        idx, f = self.operations_timeline.get_floor(time)
        if f is None:
            return super_type()

        _, rem_set = self.removal_timeline.get_floor(time)
        rem_set = set() if rem_set is None else rem_set

        out = super_type()
        for x in range(idx + 1):
            if self.operations_timeline[x].tag not in rem_set:
                out.apply_operation(self.operations_timeline[x].operation, self.operations_timeline[x].tag)

        return out

    
    t = type('TARDIS',
             (super_type, ),
             {'__init__':               init,
              'apply_operation':        apply_operation,
              'apply_operation_after':  apply_operation_after,
              'apply_operation_before': apply_operation_before,
              'remove_operation':       remove_operation,
              'get_model_at_time':      get_model_at_time})

    return t(*args)
