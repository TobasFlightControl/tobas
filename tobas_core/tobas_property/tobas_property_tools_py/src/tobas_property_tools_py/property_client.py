import rospy
import os.path as osp
from typing import Tuple, Type, Any
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from tobas_property_msgs.srv import GetBool, GetInt, GetDouble, GetString, SetBool, SetInt, SetDouble, SetString


class PropertyClient:
    # Error Code
    E_NO_ERROR = 0
    E_FAILED_TO_CONNECT = -1
    E_FAILED_TO_CALL = -2
    E_SERVER_ERROR = -3

    WAIT_FOR_SERVICE = 3.0  # [s]

    def __init__(self, ns: str, section: str) -> None:
        self._ns = ns
        self._section = section

        self._error_code = self.E_NO_ERROR
        self._server_error_msg = ""

    def get_bool(self, key: str) -> Tuple[int, bool]:
        return self._get_property(key, "get_bool", GetBool)

    def get_int(self, key: str) -> Tuple[int, int]:
        return self._get_property(key, "get_int", GetInt)

    def get_float(self, key: str) -> Tuple[int, float]:
        return self._get_property(key, "get_double", GetDouble)

    def get_string(self, key: str) -> Tuple[int, str]:
        return self._get_property(key, "get_string", GetString)

    def set_bool(self, key: str, value: bool) -> int:
        return self._set_property(key, value, "set_bool", SetBool)

    def set_int(self, key: str, value: int) -> int:
        return self._set_property(key, value, "set_int", SetInt)

    def set_float(self, key: str, value: float) -> int:
        return self._set_property(key, value, "set_double", SetDouble)

    def set_string(self, key: str, value: str) -> int:
        return self._set_property(key, value, "set_string", SetString)

    def save(self) -> int:
        client = rospy.ServiceProxy(osp.join(self._ns, "save_file"), Trigger)
        try:
            client.wait_for_service(self.WAIT_FOR_SERVICE)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = TriggerRequest
        try:
            res: TriggerResponse = client.call(req)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CALL
            return self._error_code

        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code

        self._error_code = self.E_NO_ERROR
        return self._error_code

    def error_code(self) -> int:
        return self._error_code

    def error_message(self) -> str:
        if self._error_code == self.E_NO_ERROR:
            return "No error."
        elif self._error_code == self.E_FAILED_TO_CONNECT:
            return "Failed to connect to service server."
        elif self._error_code == self.E_FAILED_TO_CALL:
            return "Failed to call service."
        elif self._error_code == self.E_SERVER_ERROR:
            return self._server_error_msg
        else:
            raise

    def _get_property(self, key: str, srv_name: str, SrvType: Type) -> Tuple[int, Any]:
        client = rospy.ServiceProxy(osp.join(self._ns, srv_name), SrvType)
        try:
            client.wait_for_service(self.WAIT_FOR_SERVICE)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = SrvType._request_class()
        req.section = self._section
        req.key = key

        try:
            res: SrvType._response_class = client.call(req)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CALL
            return self._error_code

        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code

        self._error_code = self.E_NO_ERROR
        return self._error_code, res.value

    def _set_property(self, key: str, value: Any, srv_name: str, SrvType: Type) -> int:
        client = rospy.ServiceProxy(osp.join(self._ns, srv_name), SrvType)
        try:
            client.wait_for_service(self.WAIT_FOR_SERVICE)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = SrvType._request_class()
        req.section = self._section
        req.key = key
        req.value = value

        try:
            res: SrvType._response_class = client.call(req)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CALL
            return self._error_code

        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code

        self._error_code = self.E_NO_ERROR
        return self._error_code
