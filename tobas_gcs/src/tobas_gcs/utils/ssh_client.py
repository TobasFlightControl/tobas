import os.path as osp
import paramiko
import socket
from scp import SCPClient
from typing import Tuple


class SSHClientWrapper:

    UTF_8 = "utf-8"

    HOST_NAME = "navio.local"  # ラズパイのホスト名
    PORT = 22  # SSHポート番号
    USER = "pi"  # ユーザ名
    LOGIN_PASSWORD = "raspberry"  # ログインパスワード

    def __init__(self) -> None:
        # TODO: AutoAddPolicyは脆弱なので，予めサーバーのホストキーをクライアントに登録する
        self._ssh_client = paramiko.SSHClient()
        self._ssh_client.load_system_host_keys()
        self._ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    def __del__(self) -> None:
        self._ssh_client.close()

    def connect(self) -> None:
        # TODO: SSH鍵認証，環境変数，秘密管理ツール等を使用して認証情報を安全に管理する
        try:
            self._ssh_client.connect(self.HOST_NAME, self.PORT, self.USER, self.LOGIN_PASSWORD)
        except paramiko.AuthenticationException:
            raise RuntimeError("Authentication failed. Please check your username or password.")
        except paramiko.SSHException:
            raise RuntimeError("Failed to establish an SSH connection.")
        except socket.error:
            raise RuntimeError("Could not connect to the server. Please check your network connection.")
        except Exception as e:
            raise RuntimeError(f"Unexpected error occurred: {e}")

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
        _, stdout, stderr = self._ssh_client.exec_command(command)
        stdout.channel.recv_exit_status()  # コマンドの実行結果を待つ

        output: str = stdout.read().decode(self.UTF_8)  # 標準出力
        error_output: str = stderr.read().decode(self.UTF_8)  # 標準エラー出力
        success: bool = stdout.channel.exit_status == 0

        return success, output, error_output

    def exec_command_super(self, command: str) -> Tuple[bool, str, str]:
        return self.exec_command(f"echo {self.LOGIN_PASSWORD} | sudo -S {command}")

    def scp_put(self, local_path: str, remote_path: str) -> None:
        with SCPClient(self._ssh_client.get_transport()) as scp:
            scp.put(local_path, remote_path, True)

    def scp_get(self, remote_path: str, local_path: str) -> None:
        with SCPClient(self._ssh_client.get_transport()) as scp:
            scp.get(remote_path, local_path, True)

    def sftp_read(self, remote_path: str) -> str:
        assert not remote_path.endswith("/")

        with self._ssh_client.open_sftp() as sftp:
            with sftp.file(remote_path, "r") as f:
                text = f.read().decode(self.UTF_8)
        return text

    def sftp_write(self, remote_path: str, text: str) -> None:
        assert not remote_path.endswith("/")

        with self._ssh_client.open_sftp() as sftp:
            with sftp.file(remote_path, "w") as f:
                f.write(text)

    def sftp_write_super(self, remote_path: str, text: str) -> None:
        """root権限が必要なファイルに書き込む．"""
        # 一時ファイルのパス
        tmp_path = osp.join("/tmp", osp.basename(remote_path))

        # 一時ファイルに書き込む
        self.sftp_write(tmp_path, text)

        # 一時ファイルを目的の場所に移動させる
        success, _, error_output = self.exec_command_super(f"mv {tmp_path} {remote_path}")
        if not success:
            raise RuntimeError(f"Failed to move {tmp_path} to {remote_path}: {error_output}")
