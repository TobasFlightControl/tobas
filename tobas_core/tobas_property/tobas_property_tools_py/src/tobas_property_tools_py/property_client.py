import rospy
import os.path as osp
from typing import Tuple, Type, Union, Any
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from tobas_property_msgs.srv import GetBool, GetInt, GetDouble, GetString, SetBool, SetInt, SetDouble, SetString


class PropertyClient:
    # Error Code
    E_NO_ERROR = 0
    E_FAILED_TO_CONNECT = -1
    E_FAILED_TO_CALL = -2
    E_SERVER_ERROR = -3

    GET_BOOL_SRV = "get_bool"
    GET_INT_SRV = "get_int"
    GET_DOUBLE_SRV = "get_double"
    GET_STRING_SRV = "get_string"
    SET_BOOL_SRV = "set_bool"
    SET_INT_SRV = "set_int"
    SET_DOUBLE_SRV = "set_double"
    SET_STRING_SRV = "set_string"
    SAVE_FILE_SRV = "save_file"

    def __init__(self, ns: str, section: str) -> None:
        self._ns = ns
        self._section = section

        self._error_code = self.E_NO_ERROR
        self._server_error_msg = ""

    def get_bool(self, key: str, timeout: rospy.Duration = rospy.Duration(-1)) -> Tuple[int, Union[bool, None]]:
        return self._get_property(key, self.GET_BOOL_SRV, GetBool, timeout)

    def get_int(self, key: str, timeout: rospy.Duration = rospy.Duration(-1)) -> Tuple[int, Union[int, None]]:
        return self._get_property(key, self.GET_INT_SRV, GetInt, timeout)

    def get_float(self, key: str, timeout: rospy.Duration = rospy.Duration(-1)) -> Tuple[int, Union[float, None]]:
        return self._get_property(key, self.GET_DOUBLE_SRV, GetDouble, timeout)

    def get_string(self, key: str, timeout: rospy.Duration = rospy.Duration(-1)) -> Tuple[int, Union[str, None]]:
        return self._get_property(key, self.GET_STRING_SRV, GetString, timeout)

    def set_bool(self, key: str, value: bool, timeout: rospy.Duration = rospy.Duration(-1)) -> int:
        return self._set_property(key, value, self.SET_BOOL_SRV, SetBool, timeout)

    def set_int(self, key: str, value: int, timeout: rospy.Duration = rospy.Duration(-1)) -> int:
        return self._set_property(key, value, self.SET_INT_SRV, SetInt, timeout)

    def set_float(self, key: str, value: float, timeout: rospy.Duration = rospy.Duration(-1)) -> int:
        return self._set_property(key, value, self.SET_DOUBLE_SRV, SetDouble, timeout)

    def set_string(self, key: str, value: str, timeout: rospy.Duration = rospy.Duration(-1)) -> int:
        return self._set_property(key, value, self.SET_STRING_SRV, SetString, timeout)

    def save(self, timeout: rospy.Duration = rospy.Duration(-1)) -> int:
        client = rospy.ServiceProxy(osp.join(self._ns, self.SAVE_FILE_SRV), Trigger)

        rospy.logdebug(f'Waiting for "{self.SAVE_FILE_SRV}" service server.')
        try:
            client.wait_for_service(timeout)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = TriggerRequest()

        rospy.logdebug(f'Calling "{self.SAVE_FILE_SRV}" service.')
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
            return "Failed to connect to property service server."
        elif self._error_code == self.E_FAILED_TO_CALL:
            return "Failed to call property service."
        elif self._error_code == self.E_SERVER_ERROR:
            return self._server_error_msg
        else:
            raise

    def _get_property(self, key: str, srv_name: str, SrvType: Type, timeout: rospy.Duration) -> Tuple[int, Any]:
        client = rospy.ServiceProxy(osp.join(self._ns, srv_name), SrvType)

        rospy.logdebug(f'Waiting for "{srv_name}" service server.')
        try:
            client.wait_for_service(timeout)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code, None

        req = SrvType._request_class()
        req.section = self._section
        req.key = key

        rospy.logdebug(f'Calling "{srv_name}" service.')
        try:
            res: SrvType._response_class = client.call(req)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CALL
            return self._error_code, None

        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code, None

        self._error_code = self.E_NO_ERROR
        return self._error_code, res.value

    def _set_property(self, key: str, value: Any, srv_name: str, SrvType: Type, timeout: rospy.Duration) -> int:
        client = rospy.ServiceProxy(osp.join(self._ns, srv_name), SrvType)

        rospy.logdebug(f'Waiting for "{srv_name}" service server.')
        try:
            client.wait_for_service(timeout)
        except rospy.ROSException:
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = SrvType._request_class()
        req.section = self._section
        req.key = key
        req.value = value

        rospy.logdebug(f'Calling "{srv_name}" service.')
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
