from math import pi

config = {
    "clock_freq": 10.0,

    # Approximate Time Synchronizer
    # Cannot be more than 20% of 1/clock_freq for skip checking to work
    # Why? Skip checking is done on the clock stamps.
    "msg_proximity": 0.05,
    "message_filter_q_size": 1,

    # Topic Names
    "clock_topic": "/tick",  # Periodical messages for timing
    "angle_topic": "/angle",  # Published by image2angle node in uber-cv package
    "panda_state_topic": "/franka_state_controller/franka_states",
    "state_topic": "/state",  # Combined state of robot and pendulum
    "action_topic": "/action",  # Action resulting from policy eval on state

    # Debugging
    "print_timing_info": False
}
