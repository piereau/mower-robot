# Serial connection status
# Published by serial_bridge node to indicate connection health

bool connected              # True if serial port is open and responding
float64 last_packet_time    # ROS time of last valid packet
uint32 packets_received     # Total valid packets received
uint32 packets_sent         # Total packets sent
uint32 crc_errors           # CRC validation failures
