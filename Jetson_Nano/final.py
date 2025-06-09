import cv2
import numpy as np
import pandas as pd
import universal
from collections import deque
import time
import serial
import onnxruntime as ort
import threading


def find_x_contours(hsv_frame, lower_bound, upper_bound, x, min_area):
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:x] 
    
    centers = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            centers.append((None, None))
            continue

        M = cv2.moments(contour)
        if M["m00"] == 0:  # Handle division by zero
            centers.append((None, None))
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        centers.append((cx, cy))

    # Pad with (None, None) if not enough contours
    centers += [(None, None)] * (x - len(centers))

    return centers

def compute_velocity_components(df, obj_x, obj_y):
    N = universal.velocity_sequence_length
    if len(df) < N:
        return 0.0, 0.0

    x_values = df[obj_x][-N:].values
    y_values = df[obj_y][-N:].values
    t_values = df['time'][-N:].values

    dx = x_values[-1] - x_values[0]
    dy = y_values[-1] - y_values[0]
    dt = t_values[-1] - t_values[0]

    if dt == 0:
        return 0.0, 0.0

    direction = np.array([dx, dy])
    distance = np.linalg.norm(direction)
    unit_dir = direction / distance if distance != 0 else np.array([0.0, 0.0])

    speed = distance / dt

    vel_x = np.round(unit_dir[0] * speed, 4)
    vel_y = np.round(unit_dir[1] * speed, 4)

    return vel_x, vel_y

def limit_vel(predicted_vel, curr_pos):
    accel = 5
    buffer = 0.02
    radius = universal.OBJECT_CONFIG["paddle"]["diameter"] / 2

    d_x_dash = (predicted_vel[0] ** 2) / (2 * accel)
    d_y_dash = (predicted_vel[1] ** 2) / (2 * accel)

    new_vel = predicted_vel.copy()

    # X direction limit
    if predicted_vel[0] < 0:
        if curr_pos[0] < (radius + buffer):
            new_vel[0] = 0
        else:
            if (curr_pos[0] - d_x_dash) < (radius + buffer):
                new_vel[0] = -np.sqrt(2 * accel * abs(curr_pos[0] - (radius + buffer)))
    elif predicted_vel[0] > 0:
        if (curr_pos[0]) > (universal.TABLE_CONFIG["width"] - (radius + buffer)):
            new_vel[0] = 0
        else:
            if (curr_pos[0] + d_x_dash) > (universal.TABLE_CONFIG["width"] - (radius + buffer)):
                new_vel[0] = np.sqrt(2 * accel * abs(curr_pos[0] - (universal.TABLE_CONFIG["width"] - (radius + buffer))))

    # Y direction limit
    if predicted_vel[1] < 0:
        if (curr_pos[1]) < (radius + buffer):
            new_vel[1] = 0
        else:
            if (curr_pos[1] - d_y_dash) < (radius + buffer):
                new_vel[1] = -np.sqrt(2 * accel * abs(curr_pos[1] - (radius + buffer)))
    elif predicted_vel[1] > 0:
        if (curr_pos[1]) > (universal.TABLE_CONFIG["paddle_zone"] - (radius + buffer)):
            new_vel[1] = 0
        else:
            if (curr_pos[1] + d_y_dash) > (universal.TABLE_CONFIG["paddle_zone"] - (radius + buffer)):
                new_vel[1] = np.sqrt(2 * accel * abs(curr_pos[1] - (universal.TABLE_CONFIG["paddle_zone"] - (radius + buffer))))

    return new_vel

def compute_limited_velocity(master_paddle, final_pos, max_speed=0.1):
    """
    Compute velocity vector from master_paddle to final_pos, capped at max_speed (default 0.1 m/s).
    
    Parameters:
        master_paddle (np.ndarray or list): Current position [x, y] in meters
        final_pos (np.ndarray or list): Desired position [x, y] in meters
        max_speed (float): Maximum allowed speed in m/s

    Returns:
        np.ndarray: Velocity vector [vx, vy] in m/s
    """
    master_paddle = np.array(master_paddle, dtype=float)
    final_pos = np.array(final_pos, dtype=float)

    delta = final_pos - master_paddle
    distance = np.linalg.norm(delta)

    if distance < 5e-3:
        return np.array([0.0, 0.0])

    scale = min(1.0, max_speed / distance)
    velocity = delta * scale
    return velocity

def normalize(value, center, half_range):
    return None if value is None else (value - center) / half_range

import subprocess


# Spawn ffmpeg to grab MJPEG frames from the camera
ffmpeg_cmd = [
    'ffmpeg',
    '-f', 'v4l2',
    '-framerate', '100',
    '-video_size', '640x480',
    '-input_format', 'mjpeg',
    '-i', '/dev/video0',
    '-f', 'image2pipe',
    '-vcodec', 'mjpeg',
    '-'
]

ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

latest_frame = None
lock = threading.Lock()

def frame_reader():
    global latest_frame
    buffer = b''

    while True:
        byte = ffmpeg_process.stdout.read(1024)  # Read in chunks
        if not byte:
            break
        buffer += byte
        while True:
            start = buffer.find(b'\xff\xd8')  # JPEG start
            end = buffer.find(b'\xff\xd9')    # JPEG end
            if start != -1 and end != -1 and end > start:
                jpeg_bytes = buffer[start:end+2]
                buffer = buffer[end+2:]

                jpg_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(jpg_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    with lock:
                        latest_frame = frame
            else:
                break

# Start background thread
thread = threading.Thread(target=frame_reader, daemon=True)
thread.start()

time.sleep(1)
# Function to get the most recent frame
def read_latest_frame():
    with lock:
        return latest_frame.copy() if latest_frame is not None else None

ser = serial.Serial('/dev/ttyUSB0', 115200)

prev_velocity_paddle = 0,0

print("Starting process...")

# At top of file
X_SLICE = slice(universal.x_start, universal.x_end + 1)
Y_SLICE = slice(universal.y_start, universal.y_end + 1)

X_SLICE_paddle = slice(universal.x_start_paddle, universal.x_end_paddle + 1)
Y_SLICE_paddle = slice(universal.y_start_paddle, universal.y_end_paddle + 1)

table_length, table_width, paddle_zone = universal.TABLE_CONFIG["length"], universal.TABLE_CONFIG["width"], universal.TABLE_CONFIG["paddle_zone"]
paddle_radius = universal.OBJECT_CONFIG["paddle"]["diameter"]/2
puck_radius = universal.OBJECT_CONFIG["puck"]["diameter"]/2

frame_data = deque(maxlen=universal.velocity_sequence_length)

# Load the ONNX model (only once, ideally outside the loop)
onnx_session = ort.InferenceSession("my_ppo_model.onnx", providers=["CPUExecutionProvider"])

last_time = time.time()

try:
    while True:
        predictions = [None, None]
        #read frame
        frame = read_latest_frame()
        if frame is None:
            break

        
        starttime = time.time()
        #elapsed = starttime - last_time
        #print(1/elapsed)
        #last_time = starttime

        # Crop the frame
        cropped_frame_puck = frame[Y_SLICE, X_SLICE]    
        cropped_frame_puck = cv2.rotate(cropped_frame_puck, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cropped_frame_puck_blurred = cv2.GaussianBlur(cropped_frame_puck, (11, 11), 0)

        cropped_frame_paddle = frame[Y_SLICE_paddle, X_SLICE_paddle]    
        cropped_frame_paddle = cv2.rotate(cropped_frame_paddle, cv2.ROTATE_90_COUNTERCLOCKWISE)    
        cropped_frame_paddle_blurred = cv2.GaussianBlur(cropped_frame_paddle, (7, 7), 0)

        #find the paddle center
        hsv_frame_puck = cv2.cvtColor(cropped_frame_puck_blurred, cv2.COLOR_BGR2HSV)
        hsv_frame_paddle = cv2.cvtColor(cropped_frame_paddle_blurred, cv2.COLOR_BGR2HSV)

        puck_center = find_x_contours(hsv_frame_puck, universal.puck_lower_bound, universal.puck_upper_bound, 1, 200)[0]
        master_paddle = find_x_contours(hsv_frame_paddle, universal.paddle_lower_bound, universal.paddle_upper_bound, 1, 200)[0]
        

        puck_center_cm = (None, None)
        if not None in puck_center:
            puck_center_cm = [universal.m_x_puck * puck_center[0] + universal.b_x_puck, universal.m_y_puck * puck_center[1] + universal.b_y_puck]
            puck_center_cm = np.round(puck_center_cm, 4)
            puck_center_cm[0] = np.clip(puck_center_cm[0], puck_radius, table_width - puck_radius)
            puck_center_cm[1] = np.clip(puck_center_cm[1], puck_radius, table_length - puck_radius)            

        master_paddle_cm = (None, None)
        if not None in master_paddle:
            master_paddle_cm = [universal.m_x_paddle * master_paddle[0] + universal.b_x_paddle, universal.m_y_paddle * master_paddle[1] + universal.b_y_paddle]
            master_paddle_cm = np.round(master_paddle_cm, 4)
            master_paddle_cm[0] = np.clip(master_paddle_cm[0], paddle_radius, table_width - paddle_radius)
            master_paddle_cm[1] = np.clip(master_paddle_cm[1], paddle_radius, paddle_zone - paddle_radius)

        # print(puck_center_cm)

        if None in puck_center_cm or None in master_paddle_cm:
            frame_data.clear()
        else:
            # Append clipped values
            frame_data.append([
                *puck_center_cm,
                *master_paddle_cm,
                starttime
            ])

            if (len(frame_data) == universal.velocity_sequence_length):
                

                columns = ['puck_x', 'puck_y', 'master_paddle_x', 'master_paddle_y', 'time']
                df = pd.DataFrame(frame_data, columns=columns)
                vel_puck_x, vel_puck_y = compute_velocity_components(df, "puck_x", "puck_y")
                vel_paddle_x_mean, vel_paddle_y_mean = (compute_velocity_components(df, "master_paddle_x", "master_paddle_y"))
                vel_paddle_x, vel_paddle_y = prev_velocity_paddle
                range = 0.3
                vel_paddle_x = np.clip(vel_paddle_x, vel_paddle_x_mean - range, vel_paddle_x_mean + range)
                vel_paddle_y = np.clip(vel_paddle_y, vel_paddle_y_mean - range, vel_paddle_y_mean + range)
                velocity_scale_factor = 0.1


                # Prepare normalized input as before
                input_tensor = np.array([[
                    (puck_center_cm[0] - (table_width / 2)) / (table_width / 2),
                    (puck_center_cm[1] - (table_length / 2)) / (table_length / 2),
                    (master_paddle_cm[0] - (table_width / 2)) / (table_width / 2),
                    (master_paddle_cm[1] - (table_length / 2)) / (table_length / 2),
                    vel_puck_x / universal.OBJECT_CONFIG["puck"]["max_velocity"],
                    vel_puck_y / universal.OBJECT_CONFIG["puck"]["max_velocity"],
                    vel_paddle_x / universal.OBJECT_CONFIG["paddle"]["max_velocity"],
                    vel_paddle_y / universal.OBJECT_CONFIG["paddle"]["max_velocity"]
                ]], dtype=np.float32)

                # Get the name of the first input and output for the ONNX model
                input_name = onnx_session.get_inputs()[0].name
                output_name = onnx_session.get_outputs()[0].name

                # Run inference
                output = onnx_session.run([output_name], {input_name: input_tensor})

                # `output` will be a list, typically [output_array], so extract the result
                output = output[0][0]

                output = np.clip(output, -1,1)
                predictions = output * universal.OBJECT_CONFIG["paddle"]["max_accel"]/100
                predictions = (predictions + [vel_paddle_x, vel_paddle_y])
                predictions = np.clip(predictions, -1,1)
                predictions = np.round(predictions, 3)
                predictions = limit_vel(predictions, master_paddle_cm)  
                prev_velocity_paddle = predictions          

                predictions = predictions *100

                


                # if not None in puck_center:   
                #     cv2.circle(cropped_frame_puck, puck_center, 5, (255, 0, 0), -1)
                #     cv2.putText(cropped_frame_puck, f"Puck: {puck_center_cm}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1, cv2.LINE_AA)
                #     puck_velocity_x = vel_puck_x * universal.cm_pixel * velocity_scale_factor
                #     puck_velocity_y = vel_puck_y * universal.cm_pixel * velocity_scale_factor
                #     cv2.arrowedLine(cropped_frame_puck, (int(puck_center[0]), int(puck_center[1])),
                #                         (int(puck_center[0] + puck_velocity_x), int(puck_center[1] - puck_velocity_y)),
                #                         (255, 0, 0), 2, tipLength=0.05)  # Yellow arrow for puck velocity  
                # if not None in master_paddle:
                #     cv2.circle(cropped_frame_puck, (master_paddle[0]-8, master_paddle[1]+413), 5, (0, 255, 0), -1)
                #     cv2.putText(cropped_frame_puck, f"Master Paddle: {master_paddle_cm}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1, cv2.LINE_AA)
                #     master_paddle_velocity_x = predictions[0] * universal.cm_pixel * velocity_scale_factor *0.1
                #     master_paddle_velocity_y = predictions[1] * universal.cm_pixel * velocity_scale_factor * 0.1
                #     cv2.arrowedLine(cropped_frame_puck, (int(master_paddle[0]-8), int(master_paddle[1]+413)),
                #                         (int(master_paddle[0]-8+ master_paddle_velocity_x), int(master_paddle[1]+413 - master_paddle_velocity_y)),
                #                         (0, 255, 0), 2, tipLength=0.05)  # Green arrow for master paddle velocity

                # cv2.imshow("Detected Color Centers", cropped_frame_puck)


            
                # # Press 'q' to exit
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break


        if None in predictions:

            final_pos = [table_width/2, paddle_radius*3]
            if not None in master_paddle_cm:
                predictions = compute_limited_velocity(master_paddle_cm, final_pos)
                predictions = predictions *100
            else:
                predictions = [0,0]
            # print(predictions)

        ser.reset_input_buffer()
        ser.reset_output_buffer()


        ser.write(f"{predictions[0]:.3f}, {predictions[1]:.3f}\n".encode()) 



finally:
    ffmpeg_process.terminate()
    cv2.destroyAllWindows()
    ser.close()
