import rospy
import numpy  as np
import pandas as pd

import kineverse.gradients.gradient_math  as gm

from .min_qp_builder   import TypedQPBuilder as TQPB, \
                              GeomQPBuilder  as GQPB
from kineverse.type_sets               import is_symbolic
from kineverse.time_wrapper            import Time
from kineverse.utils                   import ValueRecorder, \
                                              SymbolicRecorder
from tqdm import tqdm

DT_SYM = gm.Symbol('dt')

class CommandIntegrator(object):
    def __init__(self,
                 qp_builder,
                 integration_rules=None,
                 start_state=None,
                 recorded_terms={},
                 equilibrium=0.001,
                 printed_exprs={}):
        self.qp_builder = qp_builder
        if isinstance(qp_builder, TQPB):
            self.integration_rules = {}
            for c in qp_builder.cv:
                for s in qp_builder.free_symbols:
                    if str(gm.erase_type(s)) == str(gm.erase_type(c)):
                        t_s = gm.get_symbol_type(s)
                        t_c = gm.get_symbol_type(c)
                        if t_s < t_c:
                            self.integration_rules[s] = s + c * (DT_SYM ** (t_c - t_s))
                        elif t_s == t_c:
                            self.integration_rules[s] = s

            if integration_rules is not None:
                # Only add custom rules which are fully defined given the set of state variables and the set of command variables
                cv_set = set(self.qp_builder.cv).union({DT_SYM})
                delta_set = {s: gm.free_symbols(r).difference(self.qp_builder.free_symbols).difference(cv_set) for s, r in integration_rules.items()}
                for s, r in integration_rules.items():
                    if len(delta_set[s]) == 0:
                        self.integration_rules[s] = r
                    else:
                        print('Dropping rule "{}: {}". Symbols missing from state: {}'.format(s, r, delta_set[s]))
            # print('Integration rules:\n  {}'.format('\n  '.join(['{}: {}'.format(k, r) for k, r in sorted([(str(k), str(r)) for k, r in self.integration_rules.items()])])))
        else:
            self.integration_rules = integration_rules if integration_rules is not None else {s: s*DT_SYM for s in self.qp_builder.free_symbols}

        self.start_state = {s: 0.0 for s in self.qp_builder.free_symbols}
        if start_state is not None:
            self.start_state.update(start_state)
        self.recorded_terms = recorded_terms
        self.equilibrium = equilibrium
        self.current_iteration = 0
        self.printed_exprs = sorted(list(printed_exprs.items()))
        self._cythonized_printed_exprs = None
        self._aligned_state_vars     = None
        self._cythonized_integration = None

    def restart(self, title='Integrator'):
        self.state    = self.start_state.copy()
        self.recorder = ValueRecorder(title, *sorted([str(s) for s in self.state.keys()]))
        self.sym_recorder = SymbolicRecorder(title, **{k: gm.extract_expr(s) for k, s in self.recorded_terms.items() 
                                                                                   if is_symbolic(s)})
        self.current_iteration = 0

        self._aligned_state_vars, rules = zip(*self.integration_rules.items())
        rule_matrix = gm.Matrix([rules])
        self._cythonized_integration = gm.speed_up(rule_matrix, gm.free_symbols(rule_matrix))

        if len(self.printed_exprs) > 0:
            expr_matrix = gm.Matrix([gm.extract_expr(e) for _, e in self.printed_exprs])
            self._cythonized_printed_exprs = gm.speed_up(expr_matrix, gm.free_symbols(expr_matrix))

    @profile
    def run(self, dt=0.02, max_iterations=200, logging=True, show_progress=False, real_time=False):
        self.state[DT_SYM] = dt
        
        # Precompute geometry related values for better plots
        if isinstance(self.qp_builder, GQPB):
            self.qp_builder.compute_queries(self.state)

        cmd_accu = np.zeros(self.qp_builder.n_cv)
        temp = range(max_iterations) if not show_progress \
                                     else tqdm(range(max_iterations), desc='Running "{}" for {} iterations'.format(self.recorder.title, max_iterations))
        for x in temp:
            self.current_iteration = x
            if rospy.is_shutdown():
                break

            str_state = {str(s): v for s, v in self.state.items()}
            if logging:
                self.sym_recorder.log_symbols(self.state)
                for s, v in str_state.items():
                    if s != DT_SYM and s in self.recorder.data:
                        self.recorder.log_data(s, v)

            cmd = self.qp_builder.get_cmd(self.state, deltaT=dt)
            cmd_accu = cmd_accu * 0.5 + self.qp_builder._cmd_log[-1] * dt
            if self.qp_builder.equilibrium_reached(self.equilibrium, -self.equilibrium):
                print('Equilibrium point reached after {} iterations'.format(x))
                return

            if np.abs(cmd_accu).max() <= self.equilibrium:
                print('Lack of command signal. Aborting early...')
                return

            # print('---\n{}'.format('\n'.join(['{:>35}: {:>12.4f}'.format(k, v) for k, v in cmd.items()])))
            str_state.update({str(s): v for s, v in cmd.items()})
            new_state_vector = self._cythonized_integration(**str_state) 

            self.state.update(dict(zip(self._aligned_state_vars, new_state_vector[0])))

            # for s, i in self.integration_rules.items():
            #     update = i.subs(cmd).subs(self.state)
            #     # if s in cmd:
            #     #     print('Command for {}: {} Update: {}'.format(s, cmd[s], update))
            #     self.state[s] = update

            if self._cythonized_printed_exprs is not None:
                np_exprs = self._cythonized_printed_exprs(**str_state)
                print('Expression evaluation iteration {}:\n  {}'.format(x,
                       '\n  '.join(f'{name}: {value}' for (name, _), value in zip(self.printed_exprs, np_exprs))))

            self._post_update(dt, cmd)
            if real_time:
                Time.sleep(dt)

    def _post_update(self, dt, cmd):
        pass

    def get_latest_error(self):
        return self.qp_builder.latest_error
