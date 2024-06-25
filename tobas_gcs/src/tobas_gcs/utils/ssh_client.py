import os
import os.path as osp
import paramiko
import socket
import rospy
from scp import SCPClient
from typing import Tuple, Optional


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
        if self.is_connected():
            return

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
        try:
            _, stdout, stderr = self._ssh_client.exec_command(command)
        except AttributeError:
            return False, "", "No network connection"

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

    def scp_put_dir(self, _local_dir: str, _remote_dir: str, _exclude_dir: Optional[str] = None) -> None:
        """
        SCPでリモートディレクトリ以下にローカルディレクトリをコピーする．

        Parameters
        ----------
        _local_dir : str
            ローカルディレクトリの絶対パス．
        _remote_dir : str
            リモートディレクトリの絶対パス．
        _exclude_dir : str
            ローカルの除外するディレクトリの絶対パス．
        """
        rospy.logdebug(f"SSHClientWrapper.scp_put_dir({_local_dir}, {_remote_dir}, {_exclude_dir})")

        if not osp.isdir(_local_dir):
            raise RuntimeError(f"Local directory {_local_dir} does not exist.")

        local_dir_base = osp.basename(_local_dir.rstrip("/"))
        rospy.logdebug(f"The base name of the local directory is {local_dir_base}.")

        with SCPClient(self._ssh_client.get_transport()) as scp:
            for root, _, files in os.walk(_local_dir):
                # 指定されたディレクトリを除外
                if _exclude_dir is not None and root.startswith(_exclude_dir):
                    continue

                # ファイルを1つずつ送信
                for file in files:
                    local_file = osp.join(root, file)
                    if not osp.isfile(local_file):
                        raise RuntimeError(f"Local file {local_file} does not exist.")

                    relative_path = osp.relpath(local_file, _local_dir)
                    remote_file = osp.join(_remote_dir, local_dir_base, relative_path)
                    remote_pardir = osp.dirname(remote_file)
                    if not self.dir_exists(remote_pardir):
                        rospy.logdebug(f"Creating remote directory {remote_pardir}")
                        success, _, error_output = self.exec_command(f"mkdir -p {remote_pardir}")
                        if not success:
                            raise RuntimeError(f"Failed to create remote directory {remote_pardir}: {error_output}")

                    rospy.logdebug(f"Sending local file {local_file} into remote directory {remote_pardir}.")
                    scp.put(local_file, remote_pardir)

    def scp_put_dir_super(self, local_dir: str, remote_dir: str, exclude_dir: Optional[str] = None) -> None:
        """root権限が必要なファイルに書き込む．"""
        rospy.logdebug(f"SSHClientWrapper.scp_put_dir_super({local_dir}, {remote_dir}, {exclude_dir})")

        # リモートディレクトリが存在することを確かめる
        # 存在しなければローカルオブジェクトがそのままリモートディレクトリのパスとして配置されてしまう
        if not self.dir_exists(remote_dir):
            raise RuntimeError(f"Remote directory {remote_dir} does not exist.")

        # 一時オブジェクトに書き込む
        self.scp_put_dir(local_dir, "/tmp/", exclude_dir)

        # 一時オブジェクトのパス
        tmp_path = osp.join("/tmp", osp.basename(local_dir.rstrip("/")))

        # 一時オブジェクトをリモートディレクトリ以下にコピーする
        success, _, error_output = self.exec_command_super(f"cp -r {tmp_path} {remote_dir}")
        if not success:
            raise RuntimeError(f"Failed to move {tmp_path} to {remote_dir}: {error_output}")

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
