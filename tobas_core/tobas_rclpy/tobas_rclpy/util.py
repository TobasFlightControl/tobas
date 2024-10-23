from rclpy.duration import Duration


def seconds_from_duration(duration: Duration) -> float:
    return duration.nanoseconds / 1e9
