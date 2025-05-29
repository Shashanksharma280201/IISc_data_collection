import time

from rclpy.time import Time
from rclpy.clock import Clock, Duration


class Rate:
    def __init__(self, hz, clock: Clock = None):
        self.__hz = hz
        self.__clock = clock
        self.__time = self.now()

    def now(self):
        if self.__clock is None:
            return time.monotonic()
        return self.__clock.now().nanoseconds * 1e-9

    def sleep(self):
        sleep_duration = max(
            0,
            (1/self.__hz) - (self.now() - self.__time)
        )
        if self.__clock is None:
            time.sleep(sleep_duration)
        else:
            self.__clock.sleep_for(Duration(seconds=sleep_duration))
        self.__time = self.now()


def spin(node, executor, hz=100):
    try:
        executor.add_node(node)
        use_sim_time = node.get_parameter(
            'use_sim_time').get_parameter_value().bool_value
        # rate = Rate(hz, clock=node.get_clock())
        rate = Rate(hz)
        while executor.context.ok():
            executor.spin_once()
            if use_sim_time:
                continue
            rate.sleep()
    finally:
        executor.remove_node(node)

def lookup_tf(tf_buffer, parent_frame, child_frame, max_retry=5, logger=None):
        number_of_try = 0
        tf = None
        rate = Rate(10)
        while True:
            number_of_try += 1
            try:
                tf = tf_buffer.lookup_transform(
                    parent_frame,
                    child_frame,
                    Time()
                )
                break
            except Exception as e:
                if logger:
                    logger.warn(f"Error in looking up T_{child_frame}_{parent_frame} : {e}")
                if number_of_try == max_retry:
                    break
                rate.sleep()
        return tf
