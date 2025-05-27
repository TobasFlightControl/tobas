import rclpy
from rclpy.node import Node

from tobas_ssh_msgs.srv import Connect, Execute, List, ScpGet, ScpPut, SftpRead, SftpWrite

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

        self._connect_ss = self.create_service(Connect, "ssh/connect", self._connect_cb)
        self._execute_ss = self.create_service(Execute, "ssh/execute", self._execute_cb)
        self._scp_get_ss = self.create_service(ScpGet, "ssh/scp_get", self._scp_get_cb)
        self._scp_put_ss = self.create_service(ScpPut, "ssh/scp_put", self._scp_put_cb)
        self._sftp_read_ss = self.create_service(SftpRead, "ssh/sftp_read", self._sftp_read_cb)
        self._sftp_write_ss = self.create_service(SftpWrite, "ssh/sftp_write", self._sftp_write_cb)
        self._list_ss = self.create_service(List, "ssh/list", self._list_cb)

    def _connect(self) -> bool:
        try:
            self._ssh_client.connect()
        except Exception as e:
            self.get_logger().warning(f"{e}")
            return False

        return True

    def _connect_cb(self, req: Connect.Request, res: Connect.Response) -> Execute.Response:
        try:
            self._ssh_client.connect()
        except Exception as e:
            res.success = False
            res.message = str(e)
            return res

        res.success = True
        return res

    def _execute_cb(self, req: Execute.Request, res: Execute.Response) -> Execute.Response:
        if not self._connect():
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

    def _scp_get_cb(self, req: ScpGet.Request, res: ScpGet.Response) -> ScpGet.Response:
        if not self._connect():
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        try:
            self._ssh_client.scp_get(req.remote_path, req.local_path)
            res.success = True
        except Exception as e:
            res.success = False
            res.message = f"SCP-Get failed: {e}"

        return res

    def _scp_put_cb(self, req: ScpPut.Request, res: ScpPut.Response) -> ScpPut.Response:
        if not self._connect():
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        if req.superuser:
            try:
                self._ssh_client.scp_put_dir_super(req.local_dir, req.remote_dir, req.exclude_dirs)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SCP-Put with superuser privilege failed: {e}"
        else:
            try:
                self._ssh_client.scp_put_dir(req.local_dir, req.remote_dir, req.exclude_dirs)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SCP-Put failed: {e}"

        return res

    def _sftp_read_cb(self, req: SftpRead.Request, res: SftpRead.Response) -> SftpRead.Response:
        if not self._connect():
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        if req.superuser:
            try:
                res.text = self._ssh_client.sftp_read_super(req.remote_path)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SFTP-Read with superuser privilege failed: {e}"
        else:
            try:
                res.text = self._ssh_client.sftp_read(req.remote_path)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SFTP-Read failed: {e}"

        return res

    def _sftp_write_cb(self, req: SftpWrite.Request, res: SftpWrite.Response) -> SftpWrite.Response:
        if not self._connect():
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        if req.superuser:
            try:
                self._ssh_client.sftp_write_super(req.remote_path, req.text)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SFTP-Write with superuser privilege failed: {e}"
        else:
            try:
                self._ssh_client.sftp_write(req.remote_path, req.text)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SFTP-Write failed: {e}"

        return res

    def _list_cb(self, req: List.Request, res: List.Response) -> List.Response:
        if not self._connect():
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        try:
            res.entries = self._ssh_client.list(req.pardir)
            res.success = True
        except Exception as e:
            res.success = False
            res.message = f"Failed to list the entries in {req.pardir}: {e}"

        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SSHServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # SIGINTをキャッチして綺麗に終了
        pass


if __name__ == "__main__":
    main()
