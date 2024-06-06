import os
import os.path as osp
from configparser import ConfigParser
from typing import Optional


class ConfigParserWrapper:
    def __init__(self, config_path: str, section: str) -> None:
        self._config_path = osp.expanduser(config_path)
        self._section = section

        self._config = ConfigParser()

        # configがなければ作成
        config_dir = osp.dirname(self._config_path)
        os.makedirs(config_dir, exist_ok=True)

        # sectionがなければ作成
        self._config.read(self._config_path)
        if not self._config.has_section(section):
            self._config.add_section(section)
            self.write()

    def read(self) -> None:
        self._config.read(self._config_path)

    def write(self) -> None:
        with open(self._config_path, "w") as f:
            self._config.write(f)

    def get(self, key: str, fallback=None):
        return self._config.get(self._section, key, fallback=fallback)

    def getint(self, key: str, fallback: Optional[int] = None):
        return self._config.getint(self._section, key, fallback=fallback)

    def set(self, key: str, value) -> None:
        self._config[self._section][key] = str(value)
