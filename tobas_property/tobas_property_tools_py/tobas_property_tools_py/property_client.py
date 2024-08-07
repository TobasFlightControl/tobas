from rclpy.node import Node
from rclpy.duration import Duration
import os.path as osp
from typing import Tuple, Type, Union, Any, Optional
from std_srvs.srv import Trigger

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

    def __init__(self, node: Node, ns: str, section: str) -> None:
        self._node = node
        self._ns = ns
        self._section = section

        self._error_code = self.E_NO_ERROR
        self._server_error_msg = ""

    def get_bool(self, key: str, timeout: Optional[float] = None) -> Tuple[int, Union[bool, None]]:
        return self._get_property(key, self.GET_BOOL_SRV, GetBool, timeout)

    def get_int(self, key: str, timeout: Optional[float] = None) -> Tuple[int, Union[int, None]]:
        return self._get_property(key, self.GET_INT_SRV, GetInt, timeout)

    def get_float(self, key: str, timeout: Optional[float] = None) -> Tuple[int, Union[float, None]]:
        return self._get_property(key, self.GET_DOUBLE_SRV, GetDouble, timeout)

    def get_string(self, key: str, timeout: Optional[float] = None) -> Tuple[int, Union[str, None]]:
        return self._get_property(key, self.GET_STRING_SRV, GetString, timeout)

    def set_bool(self, key: str, value: bool, timeout: Optional[float] = None) -> int:
        return self._set_property(key, value, self.SET_BOOL_SRV, SetBool, timeout)

    def set_int(self, key: str, value: int, timeout: Optional[float] = None) -> int:
        return self._set_property(key, value, self.SET_INT_SRV, SetInt, timeout)

    def set_float(self, key: str, value: float, timeout: Optional[float] = None) -> int:
        return self._set_property(key, value, self.SET_DOUBLE_SRV, SetDouble, timeout)

    def set_string(self, key: str, value: str, timeout: Optional[float] = None) -> int:
        return self._set_property(key, value, self.SET_STRING_SRV, SetString, timeout)

    def save(self, timeout: Optional[float] = None) -> int:
        client = self._node.create_client(Trigger, osp.join(self._ns, self.SAVE_FILE_SRV))

        self._node.get_logger().debug(f'Waiting for "{self.SAVE_FILE_SRV}" service server.')
        if not client.wait_for_service(timeout):
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = Trigger.Request()

        self._node.get_logger().debug(f'Calling "{self.SAVE_FILE_SRV}" service.')
        res: Union[Trigger.Response, None] = client.call(req, timeout_sec=timeout)
        if res is None:
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

    def _get_property(self, key: str, srv_name: str, SrvType: Type, timeout: Duration) -> Tuple[int, Any]:
        client = self._node.create_client(SrvType, osp.join(self._ns, srv_name))

        self._node.get_logger().debug(f'Waiting for "{srv_name}" service server.')
        if not client.wait_for_service(timeout):
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = SrvType.Request()
        req.section = self._section
        req.key = key

        self._node.get_logger().debug(f'Calling "{srv_name}" service.')
        res: Union[SrvType.Response, None] = client.call(req, timeout_sec=timeout)
        if res is None:
            self._error_code = self.E_FAILED_TO_CALL
            return self._error_code

        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code, None

        self._error_code = self.E_NO_ERROR
        return self._error_code, res.value

    def _set_property(self, key: str, value: Any, srv_name: str, SrvType: Type, timeout: Duration) -> int:
        client = self._node.create_client(SrvType, osp.join(self._ns, srv_name))

        self._node.get_logger().debug(f'Waiting for "{srv_name}" service server.')
        if not client.wait_for_service(timeout):
            self._error_code = self.E_FAILED_TO_CONNECT
            return self._error_code

        req = SrvType.Request()
        req.section = self._section
        req.key = key
        req.value = value

        self._node.get_logger().debug(f'Calling "{srv_name}" service.')
        res: Union[SrvType.Response, None] = client.call(req, timeout_sec=timeout)
        if res is None:
            self._error_code = self.E_FAILED_TO_CALL
            return self._error_code

        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code

        self._error_code = self.E_NO_ERROR
        return self._error_code
