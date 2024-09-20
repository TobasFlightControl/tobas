import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from tobas_ssh_msgs.srv import Execute, SCPPut, SFTPRead, SFTPWrite
from .ssh_client import SSHClientWrapper


class SSHServerNode(Node):
    NO_CONNECTION_ERROR = "No connection with SSH server."

    def __init__(self) -> None:
        super().__init__("ssh_server")

        host = self.declare_parameter("host", "").get_parameter_value().string_value
        port = self.declare_parameter("port", 22).get_parameter_value().integer_value
        user = self.declare_parameter("user", "").get_parameter_value().string_value
        passwd = self.declare_parameter("passwd", "").get_parameter_value().string_value

        self._ssh_client = SSHClientWrapper(host, port, user, passwd)
        self._is_connected = False

        self._connection_pub = self.create_publisher(Bool, "ssh/connection", 1)

        self._execute_ss = self.create_service(Execute, "ssh/execute", self._execute_cb)
        self._scp_put_ss = self.create_service(SCPPut, "ssh/scp_put", self._scp_put_cb)
        self._sftp_read_ss = self.create_service(SFTPRead, "ssh/sftp_read", self._sftp_read_cb)
        self._sftp_write_ss = self.create_service(SFTPWrite, "ssh/sftp_write", self._sftp_write_cb)

        self._connect_timer = self.create_timer(1.0, self._connect_timer_cb)

    def _execute_cb(self, req: Execute.Request, res: Execute.Response) -> Execute.Response:
        if not self._is_connected:
            res.success = False
            res.error_output = self.NO_CONNECTION_ERROR
            return res

        if req.superuser:
            if req.background:
                self._ssh_client.exec_command_bg_super(req.command)
                res.success = True
                res.output = ""
                res.error_output = ""
            else:
                res.success, res.output, res.error_output = self._ssh_client.exec_command_super(req.command)
        else:
            if req.background:
                self._ssh_client.exec_command_bg(req.command)
                res.success = True
                res.output = ""
                res.error_output = ""
            else:
                res.success, res.output, res.error_output = self._ssh_client.exec_command(req.command)

        return res

    def _scp_put_cb(self, req: SCPPut.Request, res: SCPPut.Response) -> SCPPut.Response:
        if not self._is_connected:
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        if req.superuser:
            try:
                self._ssh_client.scp_put_dir_super(req.local_dir, req.remote_dir, req.exclude_dirs)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = e
        else:
            try:
                self._ssh_client.scp_put_dir(req.local_dir, req.remote_dir, req.exclude_dirs)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = e

        return res

    def _sftp_read_cb(self, req: SFTPRead.Request, res: SFTPRead.Response) -> SFTPRead.Response:
        if not self._is_connected:
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        try:
            res.text = self._ssh_client.sftp_read(req.remote_path)
            res.success = True
        except Exception as e:
            res.success = False
            res.message = e

        return res

    def _sftp_write_cb(self, req: SFTPWrite.Request, res: SFTPWrite.Response) -> SFTPWrite.Response:
        if not self._is_connected:
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        if req.superuser:
            try:
                self._ssh_client.sftp_write_super(req.remote_path, req.text)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = e
        else:
            try:
                self._ssh_client.sftp_write(req.remote_path, req.text)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = e

        return res

    def _connect_timer_cb(self) -> None:
        try:
            self._ssh_client.connect()
            self._is_connected = True
        except Exception as e:
            self.get_logger().warn(f"{e}")
            self._is_connected = False

        # Publish connection status
        connection_msg = Bool()
        connection_msg.data = self._is_connected
        self._connection_pub.publish(connection_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SSHServerNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
