from src.firmware_manager import get_board_info, upload_firmware
import pytest
import logging


@pytest.fixture
def ports_fixture():
    pass


def test_get_board_info():
    board_info: dict = get_board_info("COM3")

    assert board_info

    for key, value in board_info.items():
        assert value is not None, f"Expceted {key} to have a value, but found None."
        logging.debug(f"{key}: {value}")


def test_upload_firmware(port: str):
    for progress in upload_firmware("COM3", "arducopter.apj"):
        logging.debug(f"Progress: {progress}%")
    assert progress == 100
