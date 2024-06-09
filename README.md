# Bluetooth IMU Project

## Overview
This project is divided into two main stages, each targeting a different phase of development and usage of an IMU-based machine learning model for pose estimation and activity recognition.

### Stage 1: Training Phase - `bluetooth_imu_train`
During the initial stage, the goal is to gather high-quality, timestamped IMU data for training the machine learning model.

- **Timestamp Synchronization:** The system receives a timestamp via BLE from an iPhone to ensure synchronization with the video data.
- **IMU Data Recording:** The IMU data is recorded continuously until a stop signal is received.
- **Bulk Data Transfer:** The collected data is then transferred in bulk, preserving the temporal relationship to mitigate latency and ensure data integrity.
- **Temporal Accuracy:** This method ensures millisecond-level correlation between video and IMU data, essential for training an accurate model.

### Stage 2: Streaming Phase - `bluetooth_imu_stream`
Once the model is developed, the focus shifts to real-time pose estimation using live-streamed IMU data.

- **Real-Time Data Streaming:** The IMU data is streamed live from sensors attached to the target's wrists and ankles.
- **Model Hosting:** The trained ML model runs on a laptop, processing the live IMU data to render the target's movement in real-time.
- **Pose Estimation:** This setup provides an immediate visual representation of the person's estimated pose based solely on IMU data.

## Future Development
After the initial model correlating IMU data with video is trained, the project will extend to further refine and enhance the model:

- **Online Datasets:** Utilize existing labeled video datasets to train a secondary model.
- **Meta Movement Estimation:** This advanced model will be able to recognize and classify high-level movement patterns (e.g., weightlifting, yoga, chores) based on the IMU data.

## Project Structure
- **embedded_code/bluetooth_imu_train**
  - **Purpose:** Data collection for training.
  - **Functionality:** Timestamp synchronization, continuous data recording, bulk data transfer.

- **embedded_code/bluetooth_imu_stream**
  - **Purpose:** Real-time pose estimation.
  - **Functionality:** Live data streaming, real-time processing, immediate movement rendering.

## Hardware
- ESP32C6
- BNO085 IMU

## TODO List

### Embedded

#### Bulk Data Acquisition for Training
- [ ] Implement BLE characteristic for receiving timestamps from the iPhone.
- [ ] Record IMU data with accurate timestamps until a stop signal is received.
- [ ] Perform bulk transfer of timestamped IMU data to ensure accurate temporal relationship.

#### Live Data Streaming for Real-Time Pose Estimation
- [x] Stream IMU data via BLE for real-time processing.

### Application/Machine Learning

- **Training Stage**
  - [ ] Collect and label dataset correlating IMU data with pose information from camera feed
  - [ ] Develop and train machine learning models to map IMU data to poses seen in camera feed
  - [ ] Validate models

- **Real-Time Inference Stage**
  - [ ] Integrate trained models with the Bluetooth IMU Stream for live pose estimation.
  - [ ] Train new models which integrate human movement datasets found online for activity detection (weight lifting patterns, yoga poses, chores)


## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

 
