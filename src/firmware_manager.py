from typing import Generator, TYPE_CHECKING
from pymavlink import mavutil
from serial import Serial
import struct
import binascii
import time

if TYPE_CHECKING:
    from pymavlink.mavutil import mavserial


# protocol bytes
IN_SYNC          = b'\x12'
END_OF_CMD       = b'\x20'

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
PROGRAM_MULTIPLE_BYTES = b'\x27'
READ_MULTIPLE_BYTES = b'\x28'     # rev2 only
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

BL_REV_MIN      = 2              # minimum supported bootloader protocol
BL_REV_MAX      = 5              # maximum supported bootloader protocol

# Get info parameters
INFO_BL_REV     = b'\x01'        # bootloader protocol revision
INFO_BOARD_ID   = b'\x02'        # board type
INFO_BOARD_REV  = b'\x03'        # board revision
INFO_FLASH_SIZE = b'\x04'        # max firmware size in bytes
INFO_EXTF_SIZE  = b'\x06'        # available external flash size

PROG_MULTI_MAX  = 252            # protocol max is 255, must be multiple of 4
READ_MULTI_MAX  = 252            # protocol max is 255

NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
NSH_REBOOT_BL   = b"reboot -b\n"
NSH_REBOOT      = b"reboot\n"


def _sync(ser: Serial) -> None:
    ser.reset_input_buffer()
    ser.write(GET_SYNC + END_OF_CMD)
    ser.flush()
    reply_bytes = ser.read(2)
    if reply_bytes != IN_SYNC + OK:
        raise Exception("Got unexpected reply from the serial device:", reply_bytes)


def _get_info(ser: Serial, param: bytes):
    ser.reset_input_buffer()
    ser.write(GET_DEVICE + param + END_OF_CMD)
    ser.flush()

    info_bytes = ser.read(4)
    if len(info_bytes) < 4:
        print(info_bytes)
        raise RuntimeError("Expected to recieve 4 bytes from the buffer, but got", info_bytes)
    info = struct.unpack("<I", info_bytes)

    reply_bytes = ser.read(2)
    if reply_bytes != IN_SYNC + OK:
        print(reply_bytes)
        raise Exception("Got unexpected reply from the serial device:", reply_bytes)

    return info[0]


def _get_serial_number(ser: Serial):
    sn_word_address = [0, 4, 8]
    sn_raw = b''
    for addr in sn_word_address:
        ser.reset_input_buffer()
        ser.write(GET_SN + struct.pack("I", addr) + END_OF_CMD)
        sn_raw += ser.read(4)[::-1]
    return binascii.hexlify(sn_raw).decode()


def _get_chip_description(ser: Serial):
    ser.reset_input_buffer()
    ser.write(GET_CHIP_DES + END_OF_CMD)
    length = struct.unpack("I", ser.read(4))[0]
    desc_buf = ser.read(length)
    chip, rev = desc_buf.decode().split(',')
    return chip + " rev " + rev


def _erase_program_area(ser: Serial) -> Generator[str, None, None]:
    """ 
    Erases the program area of the serial device.
    Before calling this function, the bootloader requires that the following commands 
    are sent to the device to prevent accidental erasure:
    - GET_SYNC
    - GET_DEVICE
    """
    ser.reset_input_buffer()
    ser.write(CHIP_ERASE + END_OF_CMD)
    yield "erasing chip status: in progress"

    buf = b''
    timeout = 20
    start = time.monotonic()
    while not buf:
        if (time.monotonic() - start) > timeout:
            raise TimeoutError("Couldn't recieve a response from the board after erasing the chip")
        buf = ser.read(2)

    if buf != IN_SYNC + OK:
        raise Exception("Unexpected error occurred")
    
    yield "erasing chip status: completed"


def _write_to_program_area(ser: Serial, image: str):
    """"
    Write the firmware to the program area of the serial device.
    Before using this function, make sure the firmware image is appropriate for 
    the board by checking the board id and image size.
    """

    ser.reset_input_buffer()

    chunks: list[bytes] = []
    for i in range(0, len(image), 252):
        chunks.append(image[i: i+252])

    for chunk in chunks:
        length = len(chunk).to_bytes()
        ser.write(PROGRAM_MULTIPLE_BYTES + length + chunk + END_OF_CMD)
        if ser.read(2) != IN_SYNC + OK:
            raise Exception("Unexpected error occurred")


def _get_compatible_boards(board_id: int):
    pass

def _validate_firmware_file(file: str):
    # check board type
    # check flash size
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