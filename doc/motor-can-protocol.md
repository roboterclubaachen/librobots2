# Motor-CAN protocol

On the CAN 2.0A (Base frame format, standard identifiers) bus with 1 MBaud datarate every alpha-motor board is identified by its 8-bit ID.
The IDs are set from on a [look-up table in the Alpha-Motor code](https://git.roboterclub.rwth-aachen.de/rca/alpha-motor/blob/develop/src/alpha_motor.hpp#L102).

Three CAN messages exist.

## Sync

This message is transmitted periodically (100Hz .. 1000Hz) from the master.
The slave boards will answer with the [encoder-and-current message](#encoder-and-current-measurements).

| Identifier | Length | Data                |
|------------|--------|---------------------|
| 0x00       | 0      | (not data)          |

## Motor PWM and Current Limit

This message sets the PWM and current limit values of both motors on the alpha-motor board.

| Identifier | Length | Data[0], Data[1]    | Data[2], Data[3]    | Data[4], Data[5]         | Data[6], Data[7]         |
|------------|--------|---------------------|---------------------|--------------------------|--------------------------|
| 0x10 + ID  | 8      | int16_t PWM Motor 1 | int16_t PWM Motor 2 | uint16_t Current Motor 1 | uint16_t Current Motor 2 |


## Encoder and Current Measurements

This message is transmitted from the slave boards in response to the [sync message](#sync)

| Identifier | Length | Data[0], Data[1]         | Data[2], Data[3]         | Data[4], Data[5]        | Data[6], Data[7]        |
|------------|--------|--------------------------|--------------------------|-------------------------|-------------------------|
| 0x80 + ID  | 8      | uint16_t Encoder Motor 1 | uint16_t Encoder Motor 2 | int16_t Current Motor 1 | int16_t Current Motor 2 |

## Example Usage

### Sending messages from a Linux Host

```bash
# Setup socketcan interface
sudo ip link set can0 type can bitrate 1000000
# Enable socketcan interface
sudo ip link set can0 up

# Send a message
cansend can0 012#0ccc0000ffff0000
```
