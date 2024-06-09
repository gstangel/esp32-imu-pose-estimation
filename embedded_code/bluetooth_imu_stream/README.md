| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# ESP-IDF BLE + IMU Gatt Server Service

This project integrates a BNO085 with an esp32c6 to createa a BLE Gatt Server, and allow the trasmission of IMU Data to a client.  This data is used to train a machine learning model for pose estimation using both a camera + Pose Estimation Framework in conjunction with the IMU data, with the goal of creating a machine learning model that is capable of pose estimation using only IMU Data.
## How to Use Example

Before project configuration and build, be sure to set the correct chip target using:

```bash
idf.py set-target <chip_name>
```

### Software Required

* ESP-IDF
* esp32_BNO085 component install in $IDF_PATH/components
* see https://github.com/myles-parfeniuk/esp32_BNO08x

### Hardware Required

* A development board with ESP32/ESP32-C3/ESP32-H2/ESP32-C2/ESP32-S3 SoC (e.g., ESP32-DevKitC, ESP-WROVER-KIT, etc.)
* A USB cable for Power supply and programming
* BNO085 IMU Connected via SPI (Using a breakout board in this case)


### Build and Flash

Run `idf.py flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://idf.espressif.com/) for full steps to configure and use ESP-IDF to build projects.

## Example Output

A Gatt Server will be created, and can be connect to using a tool like Bluetility on MacOS.  There will be two characteristics, FF00 is a topic that you can subscribe to which will notify at a rate defined by imu_publish_rate_hz and will send a hex string with data in the following format:

52 bytes (Euler: X,Y,Z, Gyro: X,Y,Z, Accel: X,Y,Z, Quat: X,Y,Z,W)
ex : 01E67DC00CCABFBF3A13B0420000000000000000000000000000603E000031BF00801841008081BC000009BD00B0313F000C383F 

Euler Angles:
- X: 01E67DC0
- Y: 0CCABFBF
- Z: 3A13B042

Gyroscope:
- X: 00000000
- Y: 00000000
- Z: 0000603E

Accelerometer:
- X: 000031BF
- Y: 00801841
- Z: 008081BC

Quaternion:
- X: 000009BD
- Y: 00B0313F
- Z: 000C383F
- W: 00000000


## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
