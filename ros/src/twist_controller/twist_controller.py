
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # proposedlinearvelocity
        # proposed angular velocity
        # current linear velocity
        # dbw status
        # proposed steering angle
        # any other argument you need
        final_steering_angle = args[4]


        # Return throttle, brake, steer
        return 0.2, 0., final_steering_angle
