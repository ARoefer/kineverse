from giskardpy.symengine_wrappers    import *
from kineverse.plotting              import ValueRecorder, SymbolicRecorder
from kineverse.gradients.diff_logic  import get_symbol_type
from kineverse.motion.min_qp_builder import TypedQPBuilder as TQPB

DT_SYM = sp.symbols('T_p')

class CommandIntegrator(object):
    def __init__(self, qp_builder, integration_rules=None, start_state=None, recorded_terms={}):
        self.qp_builder = qp_builder
        if type(qp_builder) is TQPB:
            self.integration_rules = {}
            for c in qp_builder.cv:
                for s in qp_builder.free_symbols:
                    if str(s)[:-2] == str(c)[:-2]:
                        t_s = get_symbol_type(s)
                        t_c = get_symbol_type(c)
                        if t_s <= t_c:
                            self.integration_rules[s] = s + c * (DT_SYM ** (t_c - t_s))

            if integration_rules is not None:
                self.integration_rules.update({s: r for s, r in integration_rules.items() if s in self.qp_builder.free_symbols})
        else:
            self.integration_rules = integration_rules if integration_rules is not None else {s: s*DT_SYM for s in self.qp_builder.free_symbols}
        self.start_state = {s: 0.0 for s in self.qp_builder.free_symbols}
        if start_state is not None:
            self.start_state.update(start_state)
        self.recorded_terms = recorded_terms

    def restart(self, title='Integrator'):
        self.state    = self.start_state.copy()
        self.recorder = ValueRecorder(title, *[str(s) for s in self.state.keys()])
        self.sym_recorder = SymbolicRecorder(title, **self.recorded_terms)

    def run(self, dt=0.02, max_iterations=200):
        self.state[DT_SYM] = dt
        
        for x in range(max_iterations):
            self.sym_recorder.log_symbols(self.state)
            str_state = {str(s): v for s, v in self.state.items() if s != DT_SYM}
            for s, v in str_state.items():
                self.recorder.log_data(s, v)

            cmd = self.qp_builder.get_cmd(str_state)
            #print(self.qp_builder.last_matrix_str())
            #if self.qp_builder.equilibrium_reached(1e-1, -1e-1):
            #    print('Equilibrium point reached after {} iterations'.format(x))
            #    return

            #print(cmd)
            for s, i in self.integration_rules.items():
                update = i.subs(cmd).subs(self.state)
                # if s in cmd:
                #     print('Command for {}: {} Update: {}'.format(s, cmd[s], update))
                self.state[s] = update