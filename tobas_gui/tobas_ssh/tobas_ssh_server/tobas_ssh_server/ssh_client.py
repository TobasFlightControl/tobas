# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Tobas, Inc.

import os
import os.path as osp
import socket
import stat
import posixpath
import paramiko
from paramiko.config import SSH_PORT
from scp import SCPClient
from typing import Tuple, List, Callable

from .util import is_under_any


class SSHClientWrapper:
    UTF_8 = "utf-8"

    def __init__(
        self,
        host: str,
        port: int = SSH_PORT,
        user: str = None,
        passwd: str = None,
        reject_missing_host: bool = False,
    ) -> None:
        self._host = host
        self._port = port
        self._user = user
        self._passwd = passwd

        self._cli = paramiko.SSHClient()

        # Load `~/.ssh/known_hosts`.
        self._cli.load_system_host_keys()

        # Handle cases where the server public key is not included in the client known_hosts.
        if reject_missing_host:
            missing_host_policy = paramiko.RejectPolicy()
        else:
            missing_host_policy = paramiko.AutoAddPolicy()
        self._cli.set_missing_host_key_policy(missing_host_policy)

    def connect(self) -> None:
        """
        Connect to the server.

        Note
        ----
        Do not check with `is_connected()` because running a command while disconnected may raise an "SSH session not active" error.
        """
        try:
            self._cli.connect(
                hostname=self._host,
                port=self._port,
                username=self._user,
                password=self._passwd,
            )
        except paramiko.AuthenticationException as e:
            raise RuntimeError(f"Authentication failed: {e}")
        except paramiko.SSHException as e:
            raise RuntimeError(f"Failed to establish an SSH connection: {e}")
        except socket.error as e:
            raise RuntimeError(f"Failed to connect to {self._user}@{self._host}: {e}")
        except Exception as e:
            raise RuntimeError(f"Unexpected error occurred: {e}")

    def close(self) -> None:
        self._cli.close()

    def exec_command(self, command: str) -> Tuple[bool, str, str]:
        """
        Execute command.

        Parameters
        ----------
        command: str

        Returns
        -------
        success: bool
        output: str
        error_output: str
        """
        try:
            _, stdout, stderr = self._cli.exec_command(command)
        except AttributeError:
            return False, "", "No network connection."

        stdout.channel.recv_exit_status()  # Wait for the command result.

        output: str = stdout.read().decode(self.UTF_8)  # Standard output.
        error_output: str = stderr.read().decode(self.UTF_8)  # Standard error output.
        success: bool = stdout.channel.exit_status == 0

        return success, output, error_output

    def exec_command_super(self, command: str) -> Tuple[bool, str, str]:
        """Run a sudo command."""
        if command.count("'") > 0:
            raise RuntimeError('Command with superuser privilege cannot contain "\'".')

        return self.exec_command(self._sudo_command(command))

    def exec_command_bg(self, command: str) -> None:
        """Run a command in the background."""
        self._cli.exec_command(command + " &")

    def exec_command_bg_super(self, command: str) -> None:
        """Run a sudo command in the background."""
        self.exec_command_bg(self._sudo_command(command))

    def scp_get(
        self, remote_path: str, local_path: str, progress: Callable[[str, int, int], None] | None = None
    ) -> None:
        """
        Copy a remote directory under a local directory with SCP.

        Parameters
        ----------
        remote_path : str
            Absolute path of the remote directory.
        local_path : str
            Absolute path of the local directory.
        progress : Callable[[str, int, int], None]
            Progress callback.
        """
        with SCPClient(self._cli.get_transport(), progress=progress) as scp:
            scp.get(remote_path=remote_path, local_path=local_path, recursive=True)

    def scp_put_dir(
        self,
        _local_dir: str,
        _remote_dir: str,
        _exclude_dirs: List[str] = [],
        _progress: Callable[[str, int, int], None] | None = None,
    ) -> None:
        """
        Copy a local directory under a remote directory with SCP.

        Parameters
        ----------
        _local_dir : str
            Absolute path of the local directory.
        _remote_dir : str
            Absolute path of the remote directory.
        _exclude_dirs : str
            Absolute path of the local directory to exclude.
        _progress : Callable[[str, int, int], None]
            Progress callback.
        """
        if not osp.isdir(_local_dir):
            raise RuntimeError(f"Local directory {_local_dir} does not exist.")

        local_dir_base = osp.basename(_local_dir.rstrip("/"))

        with SCPClient(self._cli.get_transport(), progress=_progress) as scp:
            for root, _, files in os.walk(_local_dir):
                # Do not send excluded directories.
                if is_under_any(root, _exclude_dirs):
                    continue

                # Send files one by one.
                for file in files:
                    local_file = osp.join(root, file)
                    if not osp.isfile(local_file):
                        raise RuntimeError(f"Local file {local_file} does not exist.")

                    relative_path = osp.relpath(local_file, _local_dir)
                    remote_file = osp.join(_remote_dir, local_dir_base, relative_path)
                    remote_pardir = osp.dirname(remote_file)
                    if not self.dir_exists(remote_pardir):
                        success, _, error_output = self.exec_command(f"mkdir -p {remote_pardir}")
                        if not success:
                            raise RuntimeError(f"Failed to create remote directory {remote_pardir}: {error_output}")

                    scp.put(local_file, remote_pardir)

    def scp_put_dir_super(
        self,
        local_dir: str,
        remote_dir: str,
        exclude_dirs: List[str] = [],
        progress: Callable[[str, int, int], None] | None = None,
    ) -> None:
        """Copy a local directory under a remote directory requiring root privileges with SCP."""
        # Path of the temporary object.
        tmp_path = osp.join("/tmp", osp.basename(local_dir.rstrip("/")))

        # Delete the temporary object if it exists because paramiko does not provide rsync.
        if self.dir_exists(tmp_path):
            success, _, error_output = self.exec_command(f"rm -r {tmp_path}")
            if not success:
                raise RuntimeError(f"{tmp_path} already exists, but failed to remove it: {error_output}")

        # Write to the temporary object.
        # It is guaranteed to sync because it was deleted beforehand.
        self.scp_put_dir(local_dir, "/tmp/", exclude_dirs, progress)

        # Sync the temporary object directly under the remote directory.
        # The remote directory is created automatically if it does not exist.
        success, _, error_output = self.exec_command_super(f"rsync -rcm --delete {tmp_path} {remote_dir}")
        if not success:
            raise RuntimeError(f"Failed to move {tmp_path} to {remote_dir}: {error_output}")

    def sftp_read(self, remote_path: str) -> str:
        assert not remote_path.endswith("/")

        with self._cli.open_sftp() as sftp:
            with sftp.open(remote_path, "r") as f:
                text = f.read().decode(self.UTF_8)
        return text

    def sftp_read_super(self, remote_path: str) -> str:
        assert not remote_path.endswith("/")

        success, output, error_output = self.exec_command_super(f"cat {remote_path}")
        if not success:
            raise RuntimeError(f"Failed to cat {remote_path}: {error_output}")

        return output

    def sftp_write(self, remote_path: str, text: str) -> None:
        assert not remote_path.endswith("/")

        with self._cli.open_sftp() as sftp:
            with sftp.file(remote_path, "w") as f:
                f.write(text)

    def sftp_write_super(self, remote_path: str, text: str) -> None:
        """Write to a file requiring root privileges."""
        # Path of the temporary file.
        tmp_path = osp.join("/tmp", osp.basename(remote_path))

        # Write to the temporary file.
        self.sftp_write(tmp_path, text)

        # Move the temporary file to the target location.
        success, _, error_output = self.exec_command_super(f"mv {tmp_path} {remote_path}")
        if not success:
            raise RuntimeError(f"Failed to move {tmp_path} to {remote_path}: {error_output}")

    def list(self, remote_pardir: str) -> List[str]:
        """`ls` command."""
        with self._cli.open_sftp() as sftp:
            return sftp.listdir(remote_pardir)

    def file_exists(self, file_path: str) -> bool:
        return self.exec_command(f"[ -f {file_path} ]")[0]

    def dir_exists(self, dir_path: str) -> bool:
        return self.exec_command(f"[ -d {dir_path} ]")[0]

    def get_remote_tree_size(self, remote_path: str) -> int:
        """Get the total size [bytes] of regular files on the remote side."""
        with self._cli.open_sftp() as sftp:
            attr = sftp.stat(remote_path)

            if stat.S_ISREG(attr.st_mode):
                return int(attr.st_size)

            if not stat.S_ISDIR(attr.st_mode):
                return 0

            total = 0

            for child in sftp.listdir_attr(remote_path):
                child_path = posixpath.join(remote_path, child.filename)
                if stat.S_ISDIR(child.st_mode):
                    total += self.get_remote_tree_size(sftp, child_path)
                elif stat.S_ISREG(child.st_mode):
                    total += int(child.st_size)

        return total

    def _sudo_command(self, command: str) -> str:
        return f"echo {self._passwd} | sudo -S bash -c '{command}'"
