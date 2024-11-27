from typing import Generator

def _reboot_fmu(keep_it_in_bootloader: bool) -> None:
    pass

def _sync() -> None:
    pass

def _get_info(param: bytes):
    pass

def _get_compatible_boards(board_id: int):
    pass

def _validate_firmware_file(file: str):
    # check board type
    # check flash size
    pass

def _erase_firmware() -> None:
    pass

def _upload_firmware(file: str) -> None:
    pass

def _get_progress() -> Generator:
    pass

def _verify_firmware() -> None:
    pass

def get_board_info(port: str) -> dict:
    return {}

def upload_firmware(port: str, file):
    pass