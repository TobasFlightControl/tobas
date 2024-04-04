import threading
import ctypes
from typing import Union


class KillableThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, *args, **kwargs) -> None:
        super().__init__(group, target, name, *args, **kwargs)

    def kill(self) -> bool:
        thread_id = self._get_id()

        # スレッドIDが見つからない場合は既に落ちているということ？
        if thread_id is None:
            return False

        if ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit)) > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            return False
        
        return True

    def _get_id(self) -> Union[int, None]:
        for id, thread in threading._active.items():
            if thread is self:
                return id
        else:
            return None
