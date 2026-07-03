# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from tobas_ssh_msgs.srv import *
from tobas_ssh_msgs.action import *

from .ssh_client import SSHClientWrapper
from .scp_progress import RecursiveScpProgress
from .util import get_local_tree_size


class SSHServerNode(Node):
    NO_CONNECTION_ERROR = "No connection with SSH server."

    def __init__(self) -> None:
        super().__init__("ssh_server")

        host = self.get_parameter_or("host")
        user = self.get_parameter_or("user")

        self._cli = None
        if host.type_ == Parameter.Type.STRING and user.type_ == Parameter.Type.STRING:
            self._cli = SSHClientWrapper(host.value, user=user.value)
            self._create_ssh_services_and_actions()
            self._client_ready_info(host.value, user.value)

        self._set_endpoint_ss = self.create_service(SetEndpoint, "ssh/set_endpoint", self._set_endpoint_cb)

    def _create_ssh_services_and_actions(self) -> None:
        self._connect_ss = self.create_service(Connect, "ssh/connect", self._connect_cb)
        self._execute_ss = self.create_service(Execute, "ssh/execute", self._execute_cb)
        self._sftp_read_ss = self.create_service(SftpRead, "ssh/sftp_read", self._sftp_read_cb)
        self._sftp_write_ss = self.create_service(SftpWrite, "ssh/sftp_write", self._sftp_write_cb)
        self._list_ss = self.create_service(List, "ssh/list", self._list_cb)

        self._scp_get_as = ActionServer(self, ScpGet, "ssh/scp_get", self._scp_get_cb)
        self._scp_put_ss = ActionServer(self, ScpPut, "ssh/scp_put", self._scp_put_cb)

    def _client_ready_info(self, host: str, user: str) -> None:
        self.get_logger().info(f"SSH client initialized for {user}@{host}.")

    def _connect(self) -> bool:
        try:
            self._cli.connect()
        except Exception as e:
            self.get_logger().warning(f"{e}")
            return False

        return True

    def _set_endpoint_cb(self, req: SetEndpoint.Request, res: SetEndpoint.Response) -> SetEndpoint.Response:
        if self._cli is None:
            self._cli = SSHClientWrapper(req.host, user=req.user)
            self._create_ssh_services_and_actions()
        else:
            self._cli.close()
            self._cli = SSHClientWrapper(req.host, user=req.user)

        self._client_ready_info(req.host, req.user)
        return res

    def _connect_cb(self, req: Connect.Request, res: Connect.Response) -> Execute.Response:
        try:
            self._cli.connect()
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
                self._cli.exec_command_bg_super(req.command)
                res.success = True
                res.output = ""
                res.error_output = ""
            else:
                res.success, res.output, res.error_output = self._cli.exec_command_super(req.command)
        else:
            if req.background:
                self._cli.exec_command_bg(req.command)
                res.success = True
                res.output = ""
                res.error_output = ""
            else:
                res.success, res.output, res.error_output = self._cli.exec_command(req.command)

        return res

    def _sftp_read_cb(self, req: SftpRead.Request, res: SftpRead.Response) -> SftpRead.Response:
        if not self._connect():
            res.success = False
            res.message = self.NO_CONNECTION_ERROR
            return res

        if req.superuser:
            try:
                res.text = self._cli.sftp_read_super(req.remote_path)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SFTP-Read with superuser privilege failed: {e}"
        else:
            try:
                res.text = self._cli.sftp_read(req.remote_path)
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
                self._cli.sftp_write_super(req.remote_path, req.text)
                res.success = True
            except Exception as e:
                res.success = False
                res.message = f"SFTP-Write with superuser privilege failed: {e}"
        else:
            try:
                self._cli.sftp_write(req.remote_path, req.text)
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
            res.entries = self._cli.list(req.pardir)
            res.success = True
        except Exception as e:
            res.success = False
            res.message = f"Failed to list the entries in {req.pardir}: {e}"

        return res

    def _scp_get_cb(self, goal_handle: ServerGoalHandle) -> ScpGet.Result:
        goal: ScpGet.Goal = goal_handle.request
        result = ScpGet.Result()

        if not self._connect():
            result.error_message = self.NO_CONNECTION_ERROR
            goal_handle.abort()
            return result

        total_size = self._cli.get_remote_tree_size(goal.remote_path)

        def callback(transferred: int) -> None:
            feedback = ScpGet.Feedback()
            feedback.total_size = total_size
            feedback.transferred = min(total_size, transferred)
            goal_handle.publish_feedback(feedback)

        try:
            self._cli.scp_get(goal.remote_path, goal.local_path, RecursiveScpProgress(callback))
        except Exception as e:
            result.error_message = f"SCP-Get failed: {e}"
            goal_handle.abort()
            return result

        goal_handle.succeed()
        return result

    def _scp_put_cb(self, goal_handle: ServerGoalHandle) -> ScpPut.Result:
        goal: ScpPut.Goal = goal_handle.request
        result = ScpPut.Result()

        if not self._connect():
            result.error_message = self.NO_CONNECTION_ERROR
            goal_handle.abort()
            return result

        # Create parent directories
        if goal.parents:
            mkdir_command = f"mkdir -p {goal.remote_dir}"
            if goal.superuser:
                success, _, error_output = self._cli.exec_command_super(mkdir_command)
            else:
                success, _, error_output = self._cli.exec_command(mkdir_command)
            if not success:
                result.error_message = f"Failed to create remote directory {goal.remote_dir}: {error_output}"
                goal_handle.abort()
                return result

        total_size = get_local_tree_size(goal.local_dir)

        def callback(transferred: int) -> None:
            feedback = ScpPut.Feedback()
            feedback.total_size = total_size
            feedback.transferred = min(total_size, transferred)
            goal_handle.publish_feedback(feedback)

        progress = RecursiveScpProgress(callback)

        if goal.superuser:
            try:
                self._cli.scp_put_dir_super(goal.local_dir, goal.remote_dir, goal.exclude_dirs, progress)
            except Exception as e:
                result.error_message = f"SCP-Put with superuser privilege failed: {e}"
                goal_handle.abort()
                return
        else:
            try:
                self._cli.scp_put_dir(goal.local_dir, goal.remote_dir, goal.exclude_dirs, progress)
            except Exception as e:
                result.error_message = f"SCP-Put failed: {e}"
                goal_handle.abort()
                return

        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SSHServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # Catch SIGINT and exit cleanly.
        pass


if __name__ == "__main__":
    main()
