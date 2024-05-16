def create_empty_file(file_path: str) -> None:
    """指定されたパスにからのファイルを作成する．"""
    with open(file_path, "w"):
        pass  # ファイルに何も書き込まなければ空のファイルが作成される
