# NullModem-NG

This is a linux kernel module that provides linked pairs of serial devices that emulate the speed and flow control of standard serial devices. This is an extensive rewrite of the nullmodem module: https://github.com/pitti98/nullmodem

## Module Parameters
* burst_transfer - Controls whether multi-byte transfers are allowed. If disabled higher transmission speeds are limited by the kernel timer frequency.
* device_pairs - Number of linked pairs to create.
* tx_buffer_size - Transmission buffer size of each device.

## Todo
* Implement ioctls TIOCGSERIAL/TIOCMIWAIT.
* Implement software flow control.
