import rospy


def wait_port(input_port, wait_refresh_time=1.0, timeout=3600.0):

    waiting = True

    while waiting:
        rospy.sleep(wait_refresh_time)
        data = input_port.pop()
        waiting = data is None

    return data


def wait_port_until(input_port, expected, wait_refresh_time=1.0, timeout=3600.0):

    waiting = True

    while waiting:
        rospy.sleep(wait_refresh_time)
        received = input_port.pop()
        waiting = (received != expected)

    return True
