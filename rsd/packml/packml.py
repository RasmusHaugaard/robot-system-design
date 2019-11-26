import numpy as np


def enum_text_array(STATE):
    state_names = [name for name in dir(STATE) if not name.startswith("__")]
    state_ids = [getattr(STATE, name) for name in state_names]
    return np.array(state_names)[np.argsort(state_ids)]


class PackMLState:
    UNHOLDING = 0
    HELD = 1
    HOLDING = 2
    IDLE = 3
    STARTING = 4
    EXECUTE = 5
    COMPLETING = 6
    COMPLETE = 7
    RESETTING = 8
    UNSUSPENDING = 9
    SUSPENDED = 10
    SUSPENDING = 11
    STOPPED = 12
    STOPPING = 13
    CLEARING = 14
    ABORTED = 15
    ABORTING = 16


class PackMLActions:
    RESET = 0
    START = 1
    HOLD = 2
    UNHOLD = 3
    SUSPEND = 4
    UNSUSPEND = 5
    STOP = 6
    ABORT = 7
    CLEAR = 8


S = PackMLState
A = PackMLActions

STATE_COMPLETE_TRANSITION = {
    S.UNHOLDING: S.EXECUTE,
    S.HOLDING: S.HELD,
    S.STARTING: S.EXECUTE,
    S.EXECUTE: S.COMPLETE,
    S.COMPLETING: S.COMPLETE,
    S.RESETTING: S.IDLE,
    S.UNSUSPENDING: S.EXECUTE,
    S.SUSPENDING: S.SUSPENDED,
    S.STOPPING: S.STOPPED,
    S.CLEARING: S.STOPPED,
    S.ABORTING: S.ABORTED,
}

ACTION_TRANSITION_STATE = {
    A.RESET: S.RESETTING,
    A.START: S.STARTING,
    A.HOLD: S.HOLDING,
    A.UNHOLD: S.UNHOLDING,
    A.SUSPEND: S.SUSPENDING,
    A.UNSUSPEND: S.UNSUSPENDING,
    A.STOP: S.STOPPING,
    A.ABORT: S.ABORTING,
    A.CLEAR: S.CLEARING,
}

AVAILABLE_ACTIONS = {
    S.UNHOLDING: (A.ABORT, A.STOP),
    S.HELD: (A.ABORT, A.STOP, A.UNHOLD),
    S.HOLDING: (A.ABORT, A.STOP),
    S.IDLE: (A.ABORT, A.STOP, A.START),
    S.STARTING: (A.ABORT, A.STOP),
    S.EXECUTE: (A.ABORT, A.STOP, A.HOLD, A.SUSPEND),
    S.COMPLETING: (A.ABORT, A.STOP),
    S.COMPLETE: (A.ABORT, A.STOP, A.RESET),
    S.RESETTING: (A.ABORT, A.STOP),
    S.UNSUSPENDING: (A.ABORT, A.STOP),
    S.SUSPENDED: (A.ABORT, A.STOP, A.UNSUSPEND),
    S.SUSPENDING: (A.ABORT, A.STOP),
    S.STOPPED: (A.ABORT, A.RESET),
    S.STOPPING: (A.ABORT,),
    S.CLEARING: (A.ABORT,),
    S.ABORTED: (A.CLEAR,),
    S.ABORTING: tuple(),
}

state_names = enum_text_array(PackMLState)
action_names = enum_text_array(PackMLActions)


class PackML:
    state = S.STOPPED
    DEBUG = False

    def __init__(self, state_functions: dict = None):
        self.state_functions = state_functions if state_functions is not None else {}
        self.run()

    def run(self):
        while True:
            try:
                state_function = self.state_functions[self.state]
            except KeyError:
                state_function = lambda x: None
            res = state_function(self)
            if res is False or res is None:
                new_state = self.state
            elif res is True:
                new_state = STATE_COMPLETE_TRANSITION.get(self.state, None)
            elif type(res) == int:
                new_state = res
            else:
                raise ValueError

            if new_state == self.state:
                break
            self.state = new_state

    def __call__(self, action_id):
        if action_id in AVAILABLE_ACTIONS[self.state]:
            self.state = ACTION_TRANSITION_STATE[action_id]
        else:
            print("ACTION {} NOT AVAILABLE in STATE {}".format(
                action_names[action_id], state_names[self.state])
            )
        self.run()


def main():
    import time
    from collections import defaultdict

    def default_state_fun(pml):
        print("STATE: ", state_names[pml.state])
        time.sleep(1)

    def execute(pml):
        print("EXECUTING:: !!")
        time.sleep(5)

    state_functions = defaultdict(
        lambda: default_state_fun,
        {
            S.EXECUTE: execute,
        }
    )

    pml = PackML(state_functions)
    print("ACTION RESET")
    pml(A.RESET)
    print("ACTION START")
    pml(A.START)


if __name__ == '__main__':
    main()
