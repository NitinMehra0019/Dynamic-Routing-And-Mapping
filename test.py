import struct

# Payload format
payload_format = 'dbdbddd'

# Received payload
received_payload = b'\x00\x00\x00\x00\xd0\xd5V\xec/\xe3B@\x01\x00\x00\x00\x00\x00\x00\x00P\xfc\x18s\xd7\x9a^\xc0\xff\x00\x00\x00\x00\x00\x00\x00333333\xf3?\x00\x00\x00\x00\x00\x00\xf8?\xcd\xcc\xcc\xcc\xcc\xcc\xfc?'

# Check the length of the received payload
payload_length = len(received_payload)
print("Length of received payload:", payload_length)

# Check if the length matches the expected length for the format
expected_length = struct.calcsize(payload_format)
print("Expected length for the format:", expected_length)

if payload_length != expected_length:
    print("Length of received payload does not match the expected length.")
else:
    # Unpack the payload
    unpacked_payload = struct.unpack(payload_format, received_payload)
    print("Unpacked payload:", unpacked_payload)

