import numpy as np
import threading
import time

import pyrealsense2 as rs

import aria.sdk as aria
from projectaria_tools.core.sensor_data import ImageDataRecord
from projectaria_tools.core import calibration


class RealsenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        # Enable RGB stream
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(self.config)
        self.last_frame = np.zeros((height, width, 3), dtype=np.uint8)
        print("Realsense camera started.")

    def read(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        # Convert to numpy array
        self.last_frame = np.asanyarray(color_frame.get_data())
        return self.last_frame

    def stop(self):
        self.pipeline.stop()


class AriaCamera:
    """
    A class to handle streaming from Aria Glasses using the Aria SDK without ROS.
    Adapted from stream_aria_ros.py.
    """
    def __init__(self, device_ip=None):
        self.device_ip = device_ip
        self.last_image = None
        self.lock = threading.Lock()
        
        # 1. Setup Device Client
        aria.Level = 4
        self.device_client = aria.DeviceClient()
        client_config = aria.DeviceClientConfig()
        
        if device_ip is not None:
            print(f"Connecting to Aria device at {device_ip}")
            client_config.ip_v4_address = device_ip
        self.device_client.set_client_config(client_config)

        # 2. Connect
        self.device = self.device_client.connect()
        self.streaming_manager = self.device.streaming_manager

        # 3. Configure Streaming
        self.streaming_client = self.streaming_manager.streaming_client
        streaming_config = aria.StreamingConfig()
        streaming_config.profile_name = "profile12"
        
        if device_ip is not None:
            streaming_config.streaming_interface = aria.StreamingInterface.WifiStation
        else:
            streaming_config.streaming_interface = aria.StreamingInterface.Usb
        
        streaming_config.security_options.use_ephemeral_certs = True
        self.streaming_manager.streaming_config = streaming_config

        # 4. Handle previous sessions
        if self.streaming_manager.streaming_state.value != aria.StreamingState.NotStarted and \
           self.streaming_manager.streaming_state.value != aria.StreamingState.Stopped:
            print("Stopping Aria Streaming...")
            try:
                self.streaming_manager.stop_streaming()
            except Exception as e:
                print(f"Warning stopping stream: {e}")
        
        self.streaming_manager.start_streaming()

        # 5. Subscribe to RGB
        config = self.streaming_client.subscription_config
        config.subscriber_data_type = aria.StreamingDataType.Rgb
        self.streaming_client.subscription_config = config

        # 6. Get Calibration Data for Undistortion
        sensors_calib_json = self.streaming_manager.sensors_calibration()
        sensors_calib = calibration.device_calibration_from_json_string(sensors_calib_json)
        self.rgb_calib = sensors_calib.get_camera_calib("camera-rgb")
        
        # Pre-calculate the target linear calibration used in the reference code
        # 480x640 resolution, focal length approx logic from reference
        self.target_calib = calibration.get_linear_camera_calibration(
            480, 640, 133.25430222 * 2, "camera-rgb"
        )

        # 7. Set Observer
        # We pass 'self' to the observer so it can update the image here
        self.observer = self._StreamingObserver(self)
        self.streaming_client.set_streaming_client_observer(self.observer)
        self.streaming_client.subscribe()
        print("Aria Stream Subscribed.")

    class _StreamingObserver:
        def __init__(self, parent):
            self.parent = parent

        def on_image_received(self, image: np.array, record: ImageDataRecord):
            # This runs on the SDK thread.
            # Perform undistortion here as requested in reference logic
            try:
                undistorted = self.parent._undistort(image)
                # Thread-safe update
                with self.parent.lock:
                    self.parent.last_image = undistorted
            except Exception as e:
                print(f"Error processing Aria image: {e}")

    def _undistort(self, raw_image):
        """
        Undistort logic copied and adapted from stream_aria_ros.py
        """
        unwarped_img = calibration.distort_by_calibration(
            raw_image, self.target_calib, self.rgb_calib
        )
        # Rotate 270 degrees (k=3) as per reference
        warped_rot = np.rot90(unwarped_img, k=3)
        # The reference code converted to ROS msg here; we return the numpy array
        return warped_rot

    def read(self):
        """Return the most recent frame."""
        with self.lock:
            if self.last_image is None:
                return None
            return self.last_image.copy()

    def stop(self):
        print("Stopping Aria...")
        try:
            self.streaming_client.unsubscribe()
            if self.streaming_manager.streaming_state.value == aria.StreamingState.Streaming:
                self.streaming_manager.stop_streaming()
            self.device_client.disconnect(self.device)
        except Exception as e:
            print(f"Error closing Aria: {e}")


class ImageRecorder:
    def __init__(self, is_debug: bool = False, aria_ip: str = None):
        self.is_debug = is_debug
        
        # --- Configure Cameras ---
        
        # Cam 1: Realsense (Wrist)
        try:
            self.cam_right_wrist = RealsenseCamera()
            print("Realsense initialized.")
        except Exception as e:
            print(f"Error initializing Realsense: {e}")
            self.cam_right_wrist = None

        # Cam 2: Aria Glasses (High)
        try:
            # Pass IP if using Wifi, None if using USB
            self.cam_high = AriaCamera(device_ip=aria_ip)
            print("Aria Camera initialized.")
        except Exception as e:
            print(f"Error initializing Aria Camera: {e}")
            self.cam_high = None

        self.running = True
        self.lock = threading.Lock()
        
        self.latest_images = {
            'cam_high': None,
            'cam_right_wrist': None
        }

        # Start a thread to continuously grab frames 
        # (Though Realsense and Aria SDK have their own threads, 
        # this loop unifies the fetching for the Env)
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        time.sleep(1.0) # Warmup

    def _update(self):
        while self.running:
            # Update Wrist
            if self.cam_right_wrist:
                img_wrist = self.cam_right_wrist.read()
                if img_wrist is not None:
                    with self.lock:
                        self.latest_images['cam_right_wrist'] = img_wrist
            
            # Update High (Aria)
            if self.cam_high:
                img_high = self.cam_high.read()
                if img_high is not None:
                    with self.lock:
                        self.latest_images['cam_high'] = img_high
            
            time.sleep(0.005) # Yield

    def get_images(self):
        with self.lock:
            # Return a copy/reference safely
            return {k: (v.copy() if v is not None else None) for k, v in self.latest_images.items()}

    def stop(self):
        self.running = False
        self.thread.join()
        if self.cam_right_wrist:
            self.cam_right_wrist.stop()
        if self.cam_high:
            self.cam_high.stop()

    def print_diagnostics(self):
        imgs = self.get_images()
        for name, img in imgs.items():
            status = "OK" if img is not None else "NO DATA"
            shape = img.shape if img is not None else "N/A"
            print(f"{name}: {status} {shape}")
