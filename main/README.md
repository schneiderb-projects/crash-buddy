# BLE GATT TABLE PROTOCOL:

## GATT IDs:

- From MSB -> LSB   
    - 16 Byte ID = 2 Bytes Characteristic ID + 14 Bytes Device UUID

## GATT TABLE LAYOUT:

| MEANING  	        |CHAR ID   	|DATA TYPE          |PERMISSIONS| DESCRIPTION | 	
|---	            |---	|---                |--- |---
|STATUS             |0x0001 |bitmap - 32 bits   |READ    |0th = accelerometer connected
|DATA_AVAILABLE     |0x0002 |uint32_t           |READ    |Increment each time a new set of crash data is available
|DATA_SIZE          |0x0003 |uint32_t           |READ    |total # of data points available to be read
|SET_THRESHHOLD     |0x0004 |uint32_t           |WRITE   |Allows client to set an acceleration threshhold for determining if a crash occurred
|SET_ENABLE_DEBUG       |0x0005 |uint8_t               |WRITE   |Set to non-zero value to enable DEBUG MODE
| CRASH_DATA_CHAR_SIZE | 0x0006 | uint8_t | READ |  # of data_points per CRASH_DATA characteristic (see note below) 
|CRASH_DATA         |0x0010-0x0060 |struct data_point[124] |READ    |Accelerometer data from crash in chunks of 124 data_points (992 bytes) (see note below)

Note about CRASH_DATA Characteristics: I think characteristic data is limited to 996 bytes (1024 bytes max - 28 bytes overhead). Looking if theres a way around that, but for now, I'm thinkin I just load up a bunch of characteristics in a row with consecutive data chunks. 

```C
struct data_point {
    int32_t value; // value recorded by accelerometer
    int32_t time;  // clock time, not real time
}
```
