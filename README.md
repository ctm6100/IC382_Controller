# Humane_Controller
It is a humane version for IC382 Rescue Robot student. Full of love ~

## What's inside?
* Basic robot movement
1. int calculate_MOTOR_PWM(unsigned char percentage); => Robot PWM Calculation
2. void robot_forward(int speed); => Robot Control -> Forward
3. void robot_stop(void); => Robot Control -> Stop
4. void robot_adjust(int motor1_speed, int motor2_speed); => Robot Control -> Adjustable
* MPU6050 with FLC and Kalman Filter
1. float trimf(float measurement, float start, float peak, float end); => Fuzzy Logic: triangular membership function
2. float Rmf(float measurement, float top, float bottom); => Fuzzy Logic: R membership function
3. float Lmf(float measurement, float bottom, float top); => Fuzzy Logic: L membership function
4. float trapmf(float measurement, float start,float top_1,float top_2,float end); => Fuzzy Logic: trapezodial membership function
5. int raw_FLC(double error); => Raw fuzzy logic controller
6. int pitch_FLC(double error); => Pitch fuzzy logic controller
* DMA Receiving and Analyzing GPU Message from NVIDIA Jetson Nano
1. void gpu_msg_analyzer(void);  => Analyzing GPU message

