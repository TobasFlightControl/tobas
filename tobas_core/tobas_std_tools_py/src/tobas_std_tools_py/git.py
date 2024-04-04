import subprocess


def get_git_user_name() -> str:
    """Gitのユーザ名を返す．"""
    command = "git config --global user.name"
    result = subprocess.run(command.split(), stdout=subprocess.PIPE)
    return result.stdout.decode("utf-8").strip()


def get_git_user_email() -> str:
    """Gitのメールアドレスを返す．"""
    command = "git config --global user.email"
    result = subprocess.run(command.split(), stdout=subprocess.PIPE)
    return result.stdout.decode("utf-8").strip()
