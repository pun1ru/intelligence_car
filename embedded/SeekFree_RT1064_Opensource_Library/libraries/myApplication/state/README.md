# Vehicle modes

C26 and C27 are active-low mode switches. With both off, C14 arms balance,
C13 selects navigation, and C15 enters protection. Arming requires pitch within
10 degrees of -90 degrees. A fall beyond 45 degrees or stale attitude
locks protection until a new arm command. With both switches on, the vehicle
enters calibration mode and remains protected: C12 starts a stationary IMU
calibration and C13 resets the EKF. Holding C15 tests the left motor and
holding C14 tests the right motor, using the configured forward DIR level. If only one switch is
on, the vehicle is protected and the test PWM is zero.

Calibration takes `IMU_GYRO_CALIBRATION_TIME_MS` and is cancelled when leaving
calibration mode. Its result is applied only after enough stationary samples
have been collected. Calibration values remain in RAM until power is removed.

The state task polls buttons and serial commands and publishes the current
state to the control task. The estimate task owns EKF updates, calibration
sampling, and reset requests. The control task runs the -90 degree pitch angle
loop every 5 ms in balance and navigation, and the independent wheel-speed
test every 10 ms in calibration mode. Speed and yaw commands do not yet affect
motor output.
