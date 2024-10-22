import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize camera pipeline
pipeline = rs.pipeline()
config = rs.config()

# Configure streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
config.enable_stream(rs.stream.gyro)
config.enable_stream(rs.stream.accel)

# Align depth to color stream
align_to = rs.stream.color
align = rs.align(align_to)

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for a new frameset and align it
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        # Extract aligned frames
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        ir_frame = aligned_frames.get_infrared_frame()

        # Initialize IMU data
        gyro_data, accel_data = None, None

        # Extract motion data from frames
        for frame in frames:
            if frame.is_motion_frame():
                motion_frame = frame.as_motion_frame()
                if frame.profile.stream_type() == rs.stream.gyro:
                    gyro_data = motion_frame.get_motion_data()
                elif frame.profile.stream_type() == rs.stream.accel:
                    accel_data = motion_frame.get_motion_data()

        if not color_frame or not depth_frame or not ir_frame:
            continue  # Skip if any frame is missing

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        ir_image = np.asanyarray(ir_frame.get_data())

        # Normalize depth image for visualization
        depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)

        # Convert depth and infrared to BGR for consistency
        depth_image_bgr = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
        ir_image_bgr = cv2.cvtColor(ir_image, cv2.COLOR_GRAY2BGR)

        # Stack RGB and Depth horizontally (1280x480)
        top_row = np.hstack((color_image, depth_image_bgr))

        # Resize the infrared image to match the width of the top row (1280x480)
        ir_image_resized = cv2.resize(ir_image_bgr, (1280, 240))

        # Stack the rows vertically to create the 2x2 grid layout
        stacked_images = np.vstack((top_row, ir_image_resized))

        # Display the stacked images
        cv2.imshow('2x2 Camera Layout (RGB | Depth | Infrared)', stacked_images)

        # Print IMU data if available
        if gyro_data:
            print(f"Gyro Data: {gyro_data.x}, {gyro_data.y}, {gyro_data.z}")
        if accel_data:
            print(f"Accel Data: {accel_data.x}, {accel_data.y}, {accel_data.z}")

        # Quit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop the pipeline and close windows
    pipeline.stop()
    cv2.destroyAllWindows()
