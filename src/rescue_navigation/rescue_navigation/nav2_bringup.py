"""Bring one robot's Nav2 nodes up (configure, then activate), retrying until they are.

Nav2's lifecycle_manager does this once, node after node, and when one service call is
lost (three robots' stacks starting together load the machine) it either gives up or
waits forever, and a robot never moves. Here every tick asks each node for its state
and moves whichever is behind one step forward; a lost request only costs a retry.
"""
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
              'velocity_smoother']
STEP = {State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
        State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE}


class Nav2Bringup:
    def __init__(self, node, names=NAV2_NODES, timeout=5.0):
        self.node = node
        self.names = list(names)
        self.timeout = timeout
        self.get = {n: node.create_client(GetState, f'{n}/get_state') for n in self.names}
        self.change = {n: node.create_client(ChangeState, f'{n}/change_state') for n in self.names}
        self.state = {n: None for n in self.names}     # last known primary state id
        self.pending = {}                                # name -> (future, sent_at, kind)

    def active(self):
        return all(s == State.PRIMARY_STATE_ACTIVE for s in self.state.values())

    def refresh(self):
        """Forget what we know (e.g. Nav2 stopped answering): states are read again."""
        self.state = {n: None for n in self.names}

    def summary(self):
        names = {State.PRIMARY_STATE_UNCONFIGURED: 'unconfigured', State.PRIMARY_STATE_INACTIVE: 'inactive',
                 State.PRIMARY_STATE_ACTIVE: 'active'}
        return ', '.join(f'{n} {names.get(s, "?")}' for n, s in self.state.items())

    def tick(self, now):
        for n in self.names:
            if n in self.pending:
                fut, sent, kind = self.pending[n]
                if fut.done():
                    del self.pending[n]
                    res = fut.result()
                    if kind == 'get' and res is not None:
                        s = res.current_state.id
                        self.state[n] = s if s in (*STEP, State.PRIMARY_STATE_ACTIVE) else None
                    else:
                        self.state[n] = None     # read the state the transition left
                elif now - sent > self.timeout:
                    del self.pending[n]          # lost: ask again next tick
                continue
            s = self.state[n]
            if s is None:
                if self.get[n].service_is_ready():
                    self.pending[n] = (self.get[n].call_async(GetState.Request()), now, 'get')
            elif s in STEP and self.change[n].service_is_ready():
                req = ChangeState.Request()
                req.transition.id = STEP[s]
                self.pending[n] = (self.change[n].call_async(req), now, 'change')
