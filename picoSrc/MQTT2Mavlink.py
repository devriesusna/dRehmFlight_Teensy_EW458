import time
import ubinascii
from umqtt.simple import MQTTClient
import machine
import struct
from mavcrc import x25crc
import json


# NOTE: boot.py on Pico should contain SSID information and setup, e.g. WRCE creds

# Default  MQTT_BROKER to connect to
MQTT_BROKER = "10.24.6.23"
CLIENT_ID = ubinascii.hexlify(machine.unique_id()) # create a client ID for MQTT comms
SUBSCRIBE_TOPIC = "poseEul" # MQTT topic containing MoCap data
#PUBLISH_TOPIC = b"pitch"

# Define MAVLink message constants
MAVLINK_HEADER_LEN = 10
MAVLINK_MSG_ID_POSITION_LOCAL_NED = 32  # LOCAL_POSITION_NED ( #32 ) MESSAGE (Not Command)
MAVLINK_MSG_LEN_LOCAL_POSITION_NED = 28 # time_boot_ms (uint32_t, then 6 floats (4 bytes each))


# Setup built in PICO LED as Output
led = machine.Pin("LED",machine.Pin.OUT)

# Define UART parameters for communication with Teensy flight controller running dRehm_artemis.ino
uart = machine.UART(1, baudrate=115200, tx=machine.Pin(4), rx=machine.Pin(5),bits=8,parity=None,stop=1,rxbuf=512,timeout=100,timeout_char=5)  # UART(1) on pins 12 (TX) and 14 (RX)

# timing variables for other processes that might be happening on the pico
last_publish = time.time()
publish_interval = 0.1

# Received messages from subscriptions will be delivered to this callback
def sub_cb(topic, msg):
    print((topic, msg))
    mocapDat = json.loads(msg)
    # msg = {"name":body_name,"x":float,"y":float,"z":float,"yaw":float,"pitch":float,"roll":float}
    # data to be put in message
    x_pos = mocapDat['x'] # position
    y_pos = mocapDat['y']
    z_pos = mocapDat['z']
    vx = 0.0 # velocity
    vy = 0.0
    vz = 0.0
    
    # encode mavlink message from data
    msg = mavlink_msg_local_position_NED_encode(seqCt,x_pos, y_pos, z_pos, vx, vy, vz)
    
    # Send message over UART
    uart.write(msg)
    led.toggle()


def reset():
    print("Resetting...")
    time.sleep(5)
    machine.reset()


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



def main():
    print(f"Begin connection with MQTT Broker :: {MQTT_BROKER}")
    mqttClient = MQTTClient(CLIENT_ID, MQTT_BROKER, keepalive=60) # establish mqtt client object
    mqttClient.set_callback(sub_cb) # configure callback to send uart data when mqtt data comes in
    mqttClient.connect() # connect to broker
    mqttClient.subscribe(SUBSCRIBE_TOPIC) # subscribe to topic specified in SUBSCRIBE_TOPIC
    print(f"Connected to MQTT  Broker :: {MQTT_BROKER}, and waiting for callback function to be called!")
    while True:
            # Non-blocking wait for message
            mqttClient.check_msg()
            global last_publish
            if (time.time() - last_publish) >= publish_interval:
                # Do something else while subscriber callback happens in the background
                #mqttClient.publish(PUBLISH_TOPIC, str(random_temp).encode())
                #last_publish = time.time()


if __name__ == "__main__":
    while True:
        try:
            main()
        except OSError as e:
            print("Error: " + str(e))
            reset()
