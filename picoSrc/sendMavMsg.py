import machine
import time
import struct
from mavcrc import x25crc

# Define UART parameters (COMs to Teensy)
uart = machine.UART(1, baudrate=115200, tx=machine.Pin(4), rx=machine.Pin(5),bits=8,parity=None,stop=1,rxbuf=512,timeout=100,timeout_char=5)  # UART(1) on pins 12 (TX) and 14 (RX)

# Define MAVLink message constants
MAVLINK_HEADER_LEN = 10
MAVLINK_MSG_ID_POSITION_LOCAL_NED = 32  # LOCAL_POSITION_NED ( #32 ) MESSAGE (Not Command)
MAVLINK_MSG_LEN_LOCAL_POSITION_NED = 28 # time_boot_ms (uint32_t, then 6 floats (4 bytes each))

def mavlink_msg_local_position_NED_encode(seq,x, y, z, vx, vy, vz):
    
    # Encode position data into payload
    payload = struct.pack('<Iffffff', 2000,x, y, z, vx, vy, vz)
    
    
    # Header format
    #header[0] = 0xFD  # Start byte
    #header[1] = MAVLINK_MSG_LEN_LOCAL_POSITION_NED
    #header[2] = 0x00  # incompatibility flag
    #header[3] = 0x00 # compatbility flag
    #header[4] = seq  # Packet sequence
    #header[5] = 0x01  # System ID
    #header[6] = 0x01  # Component ID
    #header[7:9] = struct.pack('<I',MAVLINK_MSG_ID_POSITION_LOCAL_NED)  # Payload length
    #msg_id_bytes = MAVLINK_MSG_ID_POSITION_LOCAL_NED.to_bytes(2,'little')
    #header[7:9] = msg_id_bytes
    
    header = struct.pack('<BBBBBBBHB', 253, MAVLINK_MSG_LEN_LOCAL_POSITION_NED,
                               0x00, 0x00,
                               seq, 0x01, 0x01,
                               MAVLINK_MSG_ID_POSITION_LOCAL_NED&0xFFFF, MAVLINK_MSG_ID_POSITION_LOCAL_NED>>16)
    
    chkpacket = header[1:] + payload # packet that goes into checksum is header without magic byte, payload, and crc_extra byte
    crc_extra = 185 #crc_extra values found from https://github.com/okalachev/mavlink-arduino/blob/master/mavlink/common/mavlink_msg_local_position_ned.h
    
    crc = x25crc(chkpacket + bytes([crc_extra])) #https://mavlink.io/en/guide/serialization.html#crc_extra
    # removed by bradshaw 20240422
    upper_byte = (crc.crc >> 8) & 0xFF
    lower_byte = crc.crc & 0xFF        
    #print("CRC: {}, {}".format(hex(upper_byte), hex(lower_byte)))
    mavmsg = header + payload + bytes([lower_byte]) + bytes([upper_byte])
            
    return mavmsg

seqCt = 0
while True:
        
    # data to be put in message
    x_pos = 1.0 # position
    y_pos = 2.0
    z_pos = 3.0
    vx = 4.0 # velocity
    vy = 5.0
    vz = 6.0
    
    # encode mavlink message from data
    msg = mavlink_msg_local_position_NED_encode(seqCt,x_pos, y_pos, z_pos, vx, vy, vz)
    
    # Send message over UART
    uart.write(msg)
    # print packet bytes (as integers) for debugging
    integer_list = [int(byte) for byte in msg]
    print(integer_list)

    seqCt +=1 # iterate counter (not required, used to validate mavlink checksum calculation procedure
    
    time.sleep(1)
        
