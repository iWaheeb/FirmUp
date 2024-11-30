from typing import Generator, TYPE_CHECKING
from pymavlink import mavutil

if TYPE_CHECKING:
    from pymavlink.mavutil import mavserial


# protocol bytes
INSYNC          = b'\x12'
EOC             = b'\x20'

# reply bytes
OK              = b'\x10'
FAILED          = b'\x11'
INVALID         = b'\x13'     # rev3+
BAD_SILICON_REV = b'\x14'     # rev5+

# command bytes
NOP             = b'\x00'     # guaranteed to be discarded by the bootloader
GET_SYNC        = b'\x21'
GET_DEVICE      = b'\x22'
CHIP_ERASE      = b'\x23'
CHIP_VERIFY     = b'\x24'     # rev2 only
PROG_MULTI      = b'\x27'
READ_MULTI      = b'\x28'     # rev2 only
GET_CRC         = b'\x29'     # rev3+
GET_OTP         = b'\x2a'     # rev4+  , get a word from OTP area
GET_SN          = b'\x2b'     # rev4+  , get a word from SN area
GET_CHIP        = b'\x2c'     # rev5+  , get chip version
SET_BOOT_DELAY  = b'\x2d'     # rev5+  , set boot delay
GET_CHIP_DES    = b'\x2e'     # rev5+  , get chip description in ASCII
MAX_DES_LENGTH  = 20

REBOOT          = b'\x30'
SET_BAUD        = b'\x33'     # set baud

EXTF_ERASE      = b'\x34'	  # erase sectors from external flash
EXTF_PROG_MULTI = b'\x35'     # write bytes at external flash program address and increment
EXTF_READ_MULTI = b'\x36'     # read bytes at address and increment
EXTF_GET_CRC    = b'\x37'	  # compute & return a CRC of data in external flash

CHIP_FULL_ERASE = b'\x40'     # full erase of flash

INFO_BL_REV     = b'\x01'        # bootloader protocol revision
BL_REV_MIN      = 2              # minimum supported bootloader protocol
BL_REV_MAX      = 5              # maximum supported bootloader protocol
INFO_BOARD_ID   = b'\x02'        # board type
INFO_BOARD_REV  = b'\x03'        # board revision
INFO_FLASH_SIZE = b'\x04'        # max firmware size in bytes
INFO_EXTF_SIZE  = b'\x06'        # available external flash size

PROG_MULTI_MAX  = 252            # protocol max is 255, must be multiple of 4
READ_MULTI_MAX  = 252            # protocol max is 255

NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
NSH_REBOOT_BL   = b"reboot -b\n"
NSH_REBOOT      = b"reboot\n"


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

def connect(port: str) -> "mavserial":
    return mavutil.mavlink_connection(port)

def get_board_info(conn: "mavserial") -> dict:
    conn.reboot_autopilot(hold_in_bootloader= True)

    return {}

def upload_firmware(port: str, file):
    pass