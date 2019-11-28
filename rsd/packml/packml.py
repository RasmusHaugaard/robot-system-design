STATES = ("UNHOLDING", "HELD", "HOLDING", "IDLE", "STARTING", "EXECUTE", "COMPLETING", "COMPLETE", "RESETTING",
          "UNSUSPENDING", "SUSPENDED", "SUSPENDING", "STOPPED", "STOPPING", "CLEARING", "ABORTED", "ABORTING")
ACTIONS = ("RESET", "START", "HOLD", "UNHOLD", "SUSPEND", "UNSUSPEND", "STOP", "ABORT", "CLEAR")


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


class PackML:
    state = S.ABORTED

    def __init__(self, state_functions: dict = None, cb_state_change=None, debug=False):
        self.debug = debug
        self.state_functions = state_functions if state_functions is not None else {}
        self.cb_state_change = cb_state_change
        self.run()

    def set_state(self, new_state):
        if new_state != self.state:
            old_state = self.state
            self.state = new_state
            if self.cb_state_change:
                self.cb_state_change(old_state, new_state)

    def run(self):
        while True:
            if self.debug:
                print("State:", STATES[self.state])

            try:
                state_function = self.state_functions[self.state]
            except KeyError:
                state_function = lambda: None
            res = state_function()

            if res is False:
                new_state = None
            elif res is True or res is None:
                new_state = STATE_COMPLETE_TRANSITION.get(self.state, None)
            elif type(res) == int:
                new_state = res
            else:
                raise ValueError

            if new_state is None:
                break
            self.set_state(new_state)

    def __call__(self, action_id):
        if self.debug:
            print("Action:", ACTIONS[action_id])
        if action_id in AVAILABLE_ACTIONS[self.state]:
            self.set_state(ACTION_TRANSITION_STATE[action_id])
        else:
            print("ACTION {} NOT AVAILABLE in STATE {}".format(
                ACTIONS[action_id], STATES[self.state])
            )
        self.run()


def main():
    import time

    def execute():
        print("EXECUTING:: !!")
        time.sleep(3)

    def cb_state_changed(old_state, new_state):
        print("Old state: {:10} New state: {:10}".format(STATES[old_state], STATES[new_state]))

    pml = PackML({
        S.EXECUTE: execute
    }, cb_state_change=cb_state_changed)
    pml(A.CLEAR)
    pml(A.RESET)
    pml(A.START)


if __name__ == '__main__':
    main()
