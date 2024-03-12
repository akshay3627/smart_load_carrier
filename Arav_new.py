import cv2
from pyzbar.pyzbar import decode
from gpiozero import Motor, DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

# Use PiGPIOFactory for more accurate readings
factory = PiGPIOFactory()

# Define motor control pins
motor1_pin1 = 17  # GPIO pin for motor 1 input 1
motor1_pin2 = 22  # GPIO pin for motor 1 input 2

motor2_pin1 = 23  # GPIO pin for motor 2 input 1
motor2_pin2 = 24  # GPIO pin for motor 2 input 2

# Create Motor objects
motor1 = Motor(forward=motor1_pin1, backward=motor1_pin2, pin_factory=factory)
motor2 = Motor(forward=motor2_pin1, backward=motor2_pin2, pin_factory=factory)

# Define ultrasonic sensor pins
ultrasonic_echo = 18
ultrasonic_trigger = 27

# Create DistanceSensor object
ultrasonic_sensor = DistanceSensor(echo=ultrasonic_echo, trigger=ultrasonic_trigger, pin_factory=factory)

def get_qr_position(frame, points):
    # Calculate the center of the QR code
    center_x = sum(point[0] for point in points) // len(points)
    center_y = sum(point[1] for point in points) // len(points)

    # Get the width and height of the frame
    frame_height, frame_width, _ = frame.shape

    # Define regions based on the center of the QR code
    if center_x < frame_width // 3:
        position_x = "Left"
    elif center_x > 2 * frame_width // 3:
        position_x = "Right"
    else:
        position_x = "Center"

    if center_y < frame_height // 3:
        position_y = "Top"
    elif center_y > 2 * frame_height // 3:
        position_y = "Bottom"
    else:
        position_y = "Middle"

    return f"{position_y} {position_x} Corner"

def process_qr_code(frame):
    decoded_objects = decode(frame)
    for obj in decoded_objects:
        # Draw a rectangle around the QR code
        points = obj.polygon
        if len(points) > 4:
            hull = cv2.convexHull(points)
            points = hull

        num_of_points = len(points)
        for j in range(num_of_points):
            cv2.line(frame, tuple(points[j]), tuple(points[(j + 1) % num_of_points]), (0, 255, 0), 2)

        # Get the position of the QR code
        position = get_qr_position(frame, points)
        print("QR Code Position:", position)

        return position

    return None

def control_motors(qr_position, obstacle_detected):
    if obstacle_detected:
        motor1.stop()
        motor2.stop()
    elif qr_position:
        if "Top" in qr_position:
            motor1.forward()
            motor2.forward()
        elif "Center" in qr_position:
            motor1.forward()
            motor2.forward()
        elif "Bottom" in qr_position:
            motor1.backward()
            motor2.backward()
        elif "Left" in qr_position:
            motor1.forward()
            motor2.backward()
        elif "Right" in qr_position:
            motor1.backward()
            motor2.forward()
        else:
            motor1.stop()
            motor2.stop()
    else:
        motor1.stop()
        motor2.stop()

def scan_qr_code():
    cap = cv2.VideoCapture(0)
    prev_distance = 0  # Variable to store the previous distance
    obstacle_detected = False

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error capturing frame")
            break

        # Process QR codes
        position = process_qr_code(frame)

        # Check for obstacles
        distance = ultrasonic_sensor.distance * 100  # Convert to cm
        if distance < 20:
            obstacle_detected = True
        else:
            obstacle_detected = False

        # Print the ultrasonic sensor data and QR code position
        print("Ultrasonic Distance:", distance)
        if position:
            print("QR Code Position:", position)

        # Control motors based on QR code position and obstacle detection
        control_motors(position, obstacle_detected)

        # Display the frame
        cv2.imshow("QR Code Scanner", frame)

        # Break the loop when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    scan_qr_code()
