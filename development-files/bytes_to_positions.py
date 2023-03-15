import struct

# DSMX frame size (in bytes)
FRAME_SIZE = 16 * 2 + 2

# DSMX channel count
CHANNEL_COUNT = 16

# DSMX channel values
channel_values = [0] * CHANNEL_COUNT

# Byte stream to decode
byte_stream = b'1V\x00\xb2\x0c\x9c.\xaa\x138\x1c\n&\xaa'

# Ignore first two bytes (frame sync pattern)
data_bytes = byte_stream[2:]

# Iterate over channel data (2 bytes per channel)
for i in range(CHANNEL_COUNT):
    # Extract 2 bytes for the current channel
    data = data_bytes[i*2:(i+1)*2]

    # Unpack bytes as little-endian unsigned short (16-bit)
    value = struct.unpack("<H", data)[0]

    # Extract 11-bit channel value from the two bytes
    channel_values[i] = (value & 0x7FF)

print(channel_values)
