bluetooth_imu_train will be used for the inital stage of the development of the machine learning model.
- It will take a timestamp via BLE (sent using iPhone) and start recording IMU data until given a stop signal
- It will then perform a bulk transfer of the timestamped IMU data, such that there is a accurate temporal relationship
- This mitigates latency and data integrity concerns for streaming the IMU data via BLE for the training stage, where we need
   millisecond level coorelation between video and IMU data to train an accurate model


bluetooth_imu_stream will be used after the model has been developed, to give a real time rendering of a persons estimated pose
- The ML model will be hosted on a laptop, where data will be live streamed from the IMU sensors (attached to wrists and ankles of the target)
- and will provide a real time rendering of the target's movement using only IMU data

After training of the inital IMU-Video coorelation model, we can use datasets online to train another model that will coorelate movement
patterns seen in labeled videos, to get a simple estimation of the meta movements that the target is performing (weightlifting movements, yoga movements, chores, etc)