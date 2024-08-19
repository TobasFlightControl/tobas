import os.path as osp
from rclpy.node import Node
from typing import Tuple, Type, Any
from std_srvs.srv import Trigger

from tobas_property_msgs.srv import GetBool, GetInt, GetDouble, GetString, SetBool, SetInt, SetDouble, SetString


class PropertyClient:
    # Error Code
    E_NO_ERROR = 0
    E_SERVICE_NOT_READY = -1
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

    def get_bool(self, key: str) -> Tuple[int, bool | None]:
        return self._get_property(key, self.GET_BOOL_SRV, GetBool)

    def get_int(self, key: str) -> Tuple[int, int | None]:
        return self._get_property(key, self.GET_INT_SRV, GetInt)

    def get_float(self, key: str) -> Tuple[int, float | None]:
        return self._get_property(key, self.GET_DOUBLE_SRV, GetDouble)

    def get_string(self, key: str) -> Tuple[int, str | None]:
        return self._get_property(key, self.GET_STRING_SRV, GetString)

    def set_bool(self, key: str, value: bool) -> int:
        return self._set_property(key, value, self.SET_BOOL_SRV, SetBool)

    def set_int(self, key: str, value: int) -> int:
        return self._set_property(key, value, self.SET_INT_SRV, SetInt)

    def set_float(self, key: str, value: float) -> int:
        return self._set_property(key, value, self.SET_DOUBLE_SRV, SetDouble)

    def set_string(self, key: str, value: str) -> int:
        return self._set_property(key, value, self.SET_STRING_SRV, SetString)

    def save(self) -> int:
        client = self._node.create_client(Trigger, osp.join(self._ns, self.SAVE_FILE_SRV))

        self._node.get_logger().debug(f'Waiting for "{self.SAVE_FILE_SRV}" service server.')
        if not client.service_is_ready():
            self._error_code = self.E_SERVICE_NOT_READY
            return self._error_code

        req = Trigger.Request()

        self._node.get_logger().debug(f'Calling "{self.SAVE_FILE_SRV}" service.')
        res: Trigger.Response = client.call(req)
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
            return ""
        elif self._error_code == self.E_SERVICE_NOT_READY:
            return "Property server is not ready."
        elif self._error_code == self.E_FAILED_TO_CALL:
            return "Failed to call property service."
        elif self._error_code == self.E_SERVER_ERROR:
            return self._server_error_msg
        else:
            raise

    def _get_property(self, key: str, srv_name: str, SrvType: Type) -> Tuple[int, Any]:
        client = self._node.create_client(SrvType, osp.join(self._ns, srv_name))

        self._node.get_logger().debug(f'Waiting for "{srv_name}" service server.')
        if not client.service_is_ready():
            self._error_code = self.E_SERVICE_NOT_READY
            return self._error_code

        req = SrvType.Request()
        req.section = self._section
        req.key = key

        self._node.get_logger().debug(f'Calling "{srv_name}" service.')
        res: SrvType.Response = client.call(req)
        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code, None

        self._error_code = self.E_NO_ERROR
        return self._error_code, res.value

    def _set_property(self, key: str, value: Any, srv_name: str, SrvType: Type) -> int:
        client = self._node.create_client(SrvType, osp.join(self._ns, srv_name))

        self._node.get_logger().debug(f'Waiting for "{srv_name}" service server.')
        if not client.service_is_ready():
            self._error_code = self.E_SERVICE_NOT_READY
            return self._error_code

        req = SrvType.Request()
        req.section = self._section
        req.key = key
        req.value = value

        self._node.get_logger().debug(f'Calling "{srv_name}" service.')
        res: SrvType.Response = client.call(req)
        if not res.success:
            self._error_code = self.E_SERVER_ERROR
            self._server_error_msg = res.message
            return self._error_code

        self._error_code = self.E_NO_ERROR
        return self._error_code
