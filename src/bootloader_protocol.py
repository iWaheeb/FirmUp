from typing import Generator, TYPE_CHECKING, Union
from pymavlink import mavutil
from serial import Serial
from serial.tools.list_ports import comports
from base64 import b64decode
from boards import match_boards_by_id
import struct
import binascii
import time
import json
import zlib
import array

if TYPE_CHECKING:
    from pymavlink.mavutil import mavserial


# protocol bytes
IN_SYNC          = b'\x12'
END_OF_CMD       = b'\x20'

# reply bytes
OK              = b'\x10'
FAILED          = b'\x11'
INVALID         = b'\x13'

# command bytes
GET_SYNC        = b'\x21'
GET_DEVICE      = b'\x22'
CHIP_ERASE      = b'\x23'
PROGRAM_MULTIPLE_BYTES = b'\x27'
GET_CRC         = b'\x29'
GET_SN          = b'\x2b'
GET_CHIP_DES    = b'\x2e'

REBOOT          = b'\x30'
SET_BAUD        = b'\x33'     # TODO: consider using or deleting this

# GET_DEVICE parameters
INFO_BL_REV     = b'\x01'
INFO_BOARD_ID   = b'\x02'
INFO_BOARD_REV  = b'\x03'
INFO_FLASH_SIZE = b'\x04'

crc_table = array.array('I', [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d]
)
crc_padding = bytearray(b'\xff\xff\xff\xff')


def _validate_response(recv_in_sync: bytes, recv_status: bytes) -> None:
    if recv_in_sync != IN_SYNC:
        raise RuntimeError(f"Expected to recieve IN_SYNC byte, but got {recv_in_sync}")

    if recv_status == FAILED:
        raise RuntimeError("Bootloader reports operation failed")
    if recv_status == INVALID:
        raise RuntimeError("Bootloader reports invalid operation")
    if recv_status != OK:
        raise RuntimeError(f"Expected to recieve OK byte, but got {recv_status}")


def _get_sync(ser: Serial) -> None:
    ser.reset_input_buffer()
    ser.write(GET_SYNC + END_OF_CMD)
    recv_in_sync = ser.read(1)
    recv_status = ser.read(1)
    _validate_response(recv_in_sync, recv_status)


def _get_info(ser: Serial, param: bytes) -> int:
    ser.reset_input_buffer()
    ser.write(GET_DEVICE + param + END_OF_CMD)
    ser.flush()

    info_bytes = ser.read(4)
    if len(info_bytes) < 4:
        raise RuntimeError("Expected to recieve 4 bytes from the buffer, but got", info_bytes)
    info = struct.unpack("<I", info_bytes)

    recv_in_sync = ser.read(1)
    recv_status = ser.read(1)
    _validate_response(recv_in_sync, recv_status)

    return info[0]


def _get_serial_number(ser: Serial) -> str:
    sn_word_address = [0, 4, 8]
    sn_raw = b''
    for addr in sn_word_address:
        ser.reset_input_buffer()
        ser.write(GET_SN + struct.pack("I", addr) + END_OF_CMD)
        sn_raw += ser.read(4)[::-1]
        
        recv_in_sync = ser.read(1)
        recv_status = ser.read(1)
        _validate_response(recv_in_sync, recv_status)

    serial_number = binascii.hexlify(sn_raw).decode()
    return serial_number


def _get_chip_description(ser: Serial) -> str:
    ser.reset_input_buffer()
    ser.write(GET_CHIP_DES + END_OF_CMD)
    length: int = struct.unpack("I", ser.read(4))[0]
    desc_buf = ser.read(length)

    recv_in_sync = ser.read(1)
    recv_status = ser.read(1)
    _validate_response(recv_in_sync, recv_status)

    chip, rev = desc_buf.decode().split(',')
    chip_description = chip + " revision " + rev
    return chip_description


def _erase_program_area(ser: Serial) -> None:
    """ 
    Erases the program area of the serial device.
    Before calling this function, the bootloader requires that the following commands 
    are sent to the device to prevent accidental erasure:
    - GET_SYNC
    - GET_DEVICE
    """

    ser.reset_input_buffer()
    ser.write(CHIP_ERASE + END_OF_CMD)

    buf = b''
    timeout = 20
    start = time.monotonic()
    while not buf:
        if (time.monotonic() - start) > timeout:
            raise TimeoutError("Couldn't recieve a response from the board after erasing the chip")
        buf = ser.read(2)

    recv_in_sync = bytes([buf[0]])
    recv_status = bytes([buf[1]])
    _validate_response(recv_in_sync, recv_status)
    

def _write_to_program_area(ser: Serial, image: bytes) -> Generator[str, None, None]:
    """"
    Write the firmware to the program area of the serial device.
    Before using this function, make sure the firmware image is appropriate for 
    the board by checking the board id and image size.
    """

    CHUNK_SIZE = 252

    ser.reset_input_buffer()

    # ensure image length is a multiple of 4 bytes
    while ((len(image) % 4) != 0):
        image += bytes(0xFF)

    chunks: list[bytes] = []
    for i in range(0, len(image), CHUNK_SIZE):
        chunks.append(image[i: i+CHUNK_SIZE])

    sent_chunks = 0
    for chunk in chunks:
        length = len(chunk).to_bytes()
        ser.write(PROGRAM_MULTIPLE_BYTES + length + chunk + END_OF_CMD)
        recv_in_sync = ser.read(1)
        recv_status = ser.read(1)
        _validate_response(recv_in_sync, recv_status)

        sent_chunks += 1

        if sent_chunks % 100 == 0 or sent_chunks == len(chunks):
            progress = round(sent_chunks / len(chunks) * 100)
            yield  f"{progress}%"


def _get_expected_crc32(flash_size, image) -> int:
    crc_value = 0

    for byte in image:
        index = (crc_value ^ byte) & 0xff
        crc_value = crc_table[index] ^ (crc_value >> 8)

    # padding for images smaller than the flash size
    for _ in range(len(image), (flash_size - 1), 4):
        for byte in crc_padding:
            index = (crc_value ^ byte) & 0xff
            crc_value = crc_table[index] ^ (crc_value >> 8)
            
    return crc_value


def _verify_firmware(ser: Serial, flash_size: int, image: bytes) -> None:
    ser.reset_input_buffer()

    expected_crc = _get_expected_crc32(flash_size, image)

    ser.write(GET_CRC + END_OF_CMD)
    recv_crc = ser.read(4)
    recv_in_sync = ser.read(1)
    recv_status = ser.read(1)
    _validate_response(recv_in_sync, recv_status)
    
    actual_crc = struct.unpack("I", recv_crc)[0]

    if expected_crc != actual_crc:
        raise Exception(f"Verification failed. Expected crc value to be {expected_crc}, but got {actual_crc}")


def connect(selected_port: str, baudrate: int = 115200) -> Serial:
    hardware_id: str = ''
    conn: "mavserial" = None

    for port in comports():
        if port.device == selected_port:
            hardware_id = str(port.vid) + ":" + port.serial_number
            conn = mavutil.mavlink_connection(port.device)
            break

    if conn is None:
        raise Exception("couldn't find the desired serial device")
    
    conn.reboot_autopilot(True)
    conn.port.flush()
    conn.close()

    time.sleep(2)

    ser: Serial = None
    for port in comports():
        if hardware_id == str(port.vid) + ":" + port.serial_number:
            ser = Serial(port.device, baudrate, timeout=2)

    if ser is None:
        raise Exception("couldn't find the desired serial device")
    
    return ser


def get_board_info(ser: Serial) -> dict[str, Union[int, str, list[str, int]]]:
    bl_rev = _get_info(ser, INFO_BL_REV)
    board_id = _get_info(ser, INFO_BOARD_ID)
    board_rev = _get_info(ser, INFO_BOARD_REV)
    flash_size = _get_info(ser, INFO_FLASH_SIZE)
    serial_number = _get_serial_number(ser)
    chip = _get_chip_description(ser)
    boards = match_boards_by_id(board_id)

    board_info = {
        "Bootloader Revision": bl_rev,
        "Board ID": board_id,
        "Board Revision": board_rev,
        "Flash Size": flash_size,
        "Serial Number": serial_number,
        "Chip": chip,
        "Select Board": boards
    }

    return board_info


def upload_firmware(ser: Serial, path: str) -> Generator[dict[str, str], None, None]:
    
    # The bootloader requires calling GET_SYNC and GET_DEVICE before sending the
    # CHIP_ERASE command. For some unknown reason, a single GET_DEVICE call 
    # doesn't set STATE_ALLOWS_ERASE to True, so we send GET_DEVICE multiple 
    # times to ensure the correct state.
    _get_sync(ser)
    _get_info(ser, INFO_BL_REV)
    board_id = _get_info(ser, INFO_BOARD_ID)
    flash_size = _get_info(ser, INFO_FLASH_SIZE)
    
    progress = {
        "Port": ser.name,
        "Erasing Chip": "in progress",
        "Uploading Firmware": "not started",
        "Verifying Firmware": "not started"
    }
    yield progress

    _erase_program_area(ser)
    progress["Erasing Chip"] = "Completed"
    progress["Uploading Firmware"] = "0%"
    yield progress

    with open(path, "r") as file:
        data: dict = json.load(file)
        
    if data["board_id"] != board_id:
        raise RuntimeError("The provided firmware image is not suitable for this board")
    if data["image_size"] > flash_size:
        raise RuntimeError("The firmware image is too large for this board")

    encoded_image = data["image"]
    image = zlib.decompress(b64decode(encoded_image))

    for prog in _write_to_program_area(ser, image):
        progress["Uploading Firmware"] = prog
        yield progress

    progress["Verifying Firmware"] = "in progress"
    yield progress
    _verify_firmware(ser, flash_size, image)
    progress["Verifying Firmware"] = "Completed"
    yield progress

    ser.write(REBOOT + END_OF_CMD)
