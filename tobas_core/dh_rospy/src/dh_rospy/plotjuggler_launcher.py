import subprocess
import rospy
from rospy.msg import AnyMsg
from typing import List, Dict


class PlotJugglerLauncher:
    """必要なトピックを待ってからPlotJugglerを起動する．"""

    UPDATE_RATE = 1  # [Hz]
    INFO_PERIOD = 5  # [s]

    def __init__(self) -> None:
        # rosparams
        self._topics: List[str] = rospy.get_param("~required_topics", [])
        self._layout: str = rospy.get_param("~layout", "")

        self._received_msgs: Dict[str, bool] = {topic: False for topic in self._topics}

        self._subscribers: List[rospy.Subscriber] = []
        for topic in self._topics:
            sub = rospy.Subscriber(topic, AnyMsg, self._callback, callback_args=topic)
            self._subscribers.append(sub)

    def run(self) -> None:
        rate = rospy.Rate(self.UPDATE_RATE)

        while not rospy.is_shutdown():
            all_received = True

            for topic, received in self._received_msgs.items():
                if not received:
                    all_received = False
                    rospy.loginfo_throttle(
                        self.INFO_PERIOD,
                        f"Waiting for {rospy.get_namespace()}{topic}",
                    )

            if all_received:
                rospy.loginfo(
                    f"All required messages are received. Launching PlotJuggler."
                )

                # Unregister all subscribers
                for sub in self._subscribers:
                    sub.unregister()

                # Launch PlotJuggler
                if self._layout:
                    subprocess.run(
                        f"rosrun plotjuggler plotjuggler -l {self._layout}", shell=True
                    )
                else:
                    subprocess.run("rosrun plotjuggler plotjuggler", shell=True)

                return

            rate.sleep()

    def _callback(self, _: AnyMsg, topic: str) -> None:
        self._received_msgs[topic] = True
