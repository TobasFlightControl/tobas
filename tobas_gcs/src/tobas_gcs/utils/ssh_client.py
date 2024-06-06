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

    def close(self) -> None:
        self._ssh_client.close()

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
        """sudoコマンドを実行．"""
        assert command.count("'") == 0
        return self.exec_command(self._sudo_command(command))

    def exec_command_bg(self, command: str) -> None:
        """バックグラウンドでコマンドを実行．"""
        self._ssh_client.exec_command(command + " &")

    def exec_command_bg_super(self, command: str) -> None:
        """バックグラウンドでsudoコマンドを実行．"""
        self.exec_command_bg(self._sudo_command(command))

    def scp_put(self, local_path: str, remote_dir: str) -> None:
        # SCPでリモートディレクトリ以下にローカルオブジェクトをコピー
        with SCPClient(self._ssh_client.get_transport()) as scp:
            scp.put(local_path, remote_dir, True)

    def scp_put_super(self, local_path: str, remote_dir: str) -> None:
        """root権限が必要なファイルに書き込む．"""
        # リモートディレクトリが存在することを確かめる
        # 存在しなければローカルオブジェクトがそのままリモートディレクトリのパスとして配置されてしまう
        if not self.dir_exists(remote_dir):
            raise RuntimeError(f"Remote directory {remote_dir} does not exist.")

        # 一時オブジェクトに書き込む
        self.scp_put(local_path, "/tmp/")

        # 一時オブジェクトのパス
        tmp_path = osp.join("/tmp", osp.basename(local_path.rstrip("/")))

        # 一時オブジェクトをリモートディレクトリ以下にコピーする
        success, _, error_output = self.exec_command_super(f"cp -r {tmp_path} {remote_dir}")
        if not success:
            raise RuntimeError(f"Failed to move {tmp_path} to {remote_dir}: {error_output}")

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

    def file_exists(self, file_path: str) -> bool:
        return self.exec_command(f"[ -f {file_path} ]")[0]

    def dir_exists(self, dir_path: str) -> bool:
        return self.exec_command(f"[ -d {dir_path} ]")[0]

    def is_connected(self) -> bool:
        return self.exec_command("ls")[0]

    def _sudo_command(self, command: str) -> str:
        return f"echo {self.LOGIN_PASSWORD} | sudo -S bash -c '{command}'"
