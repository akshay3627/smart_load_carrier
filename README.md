# Smart Load Carrier

This project implements a robot capable of detecting QR codes and navigating based on their position. The robot also avoids obstacles using an ultrasonic sensor. The motors are controlled via GPIO on a Raspberry Pi.

## Features

- **QR Code Scanning**: Identifies the position of QR codes within the camera's frame and determines navigation directions.
- **Obstacle Avoidance**: Uses an ultrasonic sensor to detect obstacles and stop the robot to prevent collisions.
- **Motor Control**: Controls two DC motors to navigate in different directions based on QR code position.

## Requirements

- Raspberry Pi (with GPIO pins)
- Camera (e.g., USB Webcam)
- Two DC motors with a motor driver
- Ultrasonic distance sensor (e.g., HC-SR04)
- Python 3.x
- Libraries: `opencv-python`, `pyzbar`, `gpiozero`

## Installation

1. Clone this repository to your Raspberry Pi:

   ```bash
   git clone https://github.com/yourusername/qr-code-guided-robot.git
   cd qr-code-guided-robot
   ```

2. Install the required libraries:

   ```bash
   pip install opencv-python pyzbar gpiozero
   ```

3. Connect the hardware:
   - **Motors**: Connect the motors to GPIO pins 17, 22 (motor1) and 23, 24 (motor2).
   - **Ultrasonic Sensor**: Connect the trigger pin to GPIO 27 and the echo pin to GPIO 18.
   - **Webcam**: Attach a USB webcam to the Raspberry Pi.

4. Ensure proper power supply to the motors and the Raspberry Pi.

## Usage

1. Run the script:

   ```bash
   python qr_code_guided_robot.py
   ```

2. The robot will:
   - Scan for QR codes using the webcam.
   - Navigate based on the QR code's position (e.g., move forward, backward, left, or right).
   - Stop when an obstacle is detected within 20 cm.

## How It Works

1. **QR Code Detection**:
   - The camera captures real-time video.
   - `pyzbar` is used to decode QR codes.
   - The position of the QR code is calculated and classified as `Top`, `Center`, `Bottom`, `Left`, or `Right`.

2. **Obstacle Detection**:
   - The ultrasonic sensor measures the distance to obstacles.
   - If an obstacle is detected within 20 cm, the robot stops.

3. **Motor Control**:
   - Based on the QR code's position, the motors are controlled to move forward, backward, or turn left/right.

## File Structure

- `qr_code_guided_robot.py`: Main script for controlling the robot.
- `README.md`: Project documentation.

## Customization

- **Obstacle Threshold**: Adjust the ultrasonic distance threshold:
  ```python
  if distance < 20:
  ```

- **Motor Speed**: Modify the motor control logic to set different speeds:
  ```python
  motor1.forward(speed)
  motor2.forward(speed)
  ```

- **QR Code Position**: Customize navigation logic based on QR code positions in `control_motors()`.

## Safety Tips

- Ensure all connections are secure before running the robot.
- Test the obstacle detection threshold in a safe environment.
- Avoid running the robot near fragile or dangerous objects.

## Contribution

Feel free to submit issues and pull requests to enhance the project.

## License

This project is licensed under the [MIT License](LICENSE).

