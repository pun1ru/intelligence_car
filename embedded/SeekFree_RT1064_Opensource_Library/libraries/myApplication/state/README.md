# Vehicle modes

C26 and C27 are active-low mode switches. With both off, C14 arms balance,
C13 selects navigation, and C15 enters protection. A fall or stale attitude
locks protection until a new arm command. With both switches on, the vehicle
enters calibration mode and remains protected: C12 starts a stationary IMU
calibration and C13 resets the EKF. If only one switch is on, the vehicle is
protected and no calibration action is accepted.

Calibration takes `IMU_GYRO_CALIBRATION_TIME_MS` and is cancelled when leaving
calibration mode. Its result is applied only after enough stationary samples
have been collected. Calibration values remain in RAM until power is removed.

The state task polls buttons and serial commands. The estimate task owns EKF
updates, calibration sampling, and reset requests. The control task still
commands zero PWM; balance and navigation control are not connected yet.
