# Motor and sensor status from Arduino
# Matches telemetry packet format from protocol.h

int32 encoder_left          # Cumulative left encoder ticks
int32 encoder_right         # Cumulative right encoder ticks
bool watchdog_triggered     # True if watchdog fired (comms timeout)
bool estop_active           # True if E-stop is active
bool crc_error_seen         # True if Arduino saw CRC errors
uint16 battery_mv           # Battery voltage in millivolts
uint8 sequence              # Packet sequence number
