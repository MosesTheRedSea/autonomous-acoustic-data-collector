#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from audition_msgs.msg import CollectionStatus
import subprocess
import os
import time
import numpy as np
import sounddevice as sd
import soundfile as sf
from scipy.signal import fftconvolve
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt
import threading

# python3 -c "import sounddevice as sd; print(sd.query_devices())" 

class AcousticRecorder(Node):

    def __init__(self):

        super().__init__('acoustic_recorder')

        self.declare_parameter('excitation_path', '/home/moses/moses-research/ros2-robot-audition-suite/excitation.wav')
        self.declare_parameter('output_dir', '/home/moses/audition_bags/acoustic')

        self.declare_parameter('dataset_directory', '/home/moses/audition_bags/acoustic')
        self.declare_parameter('channels', 16)
        self.declare_parameter('repeat', 8)

        self.declare_parameter('sleep_duration', 3)
        self.declare_parameter('start_sample', 4900)
        self.declare_parameter('end_sample', 6000)

        # 18 Array Microphone
        self.declare_parameter('mic_device', 1)
        self.declare_parameter('speaker_device', 13)

        self.excitation_path = self.get_parameter('excitation_path').value
        self.output_dir = self.get_parameter('output_dir').value
        self.dataset_directory = self.get_parameter('dataset_directory').value

        self.channels = self.get_parameter('channels').value
        self.repeat = self.get_parameter('repeat').value
        self.sleep_duration = self.get_parameter('sleep_duration').value
        self.start_sample = self.get_parameter('start_sample').value
        self.end_sample = self.get_parameter('end_sample').value

        # Retreive the devices from the yaml file
        self.mic_device = self.get_parameter('mic_device').value
        self.speaker_device = self.get_parameter('speaker_device').value

        self.recording  = False
        self.current_waypoint = None
        self.excitation = None
        self.fs = None

        self.room_directory = "nakadai-lab"
        self.load_excitation()

        # subscribers``
        self.trigger_sub = self.create_subscription( Bool, '/start_record', self.trigger_callback, 10)
        self.waypoint_sub = self.create_subscription(CollectionStatus, '/current_waypoint', self.waypoint_callback, 10)

        # publishers
        self.complete_pub = self.create_publisher(Bool, '/recording_complete', 10)
        self.status_pub   = self.create_publisher(String, '/acoustic_status', 10)

        self.get_logger().info('Acoustic recorder ready')
        self.get_logger().info(f'Excitation: {self.excitation_path}')
        self.get_logger().info(f'Output dir: {self.output_dir}')
        self.get_logger().info(f'Channels: {self.channels} | Repeat: {self.repeat}')
        self.list_audio_devices()

    def load_excitation(self):
        if not self.excitation_path or not os.path.exists(self.excitation_path):
            self.get_logger().warn(f'Excitation file not found: {self.excitation_path}')
            return
        excitation, fs = sf.read(self.excitation_path)
        if excitation.ndim > 1:
            excitation = excitation[:, 0]
        self.excitation = excitation / np.max(np.abs(excitation))
        self.fs = fs
        self.get_logger().info(f'Excitation loaded — {len(excitation)} samples at {fs} Hz')
    def list_audio_devices(self):
        devices = sd.query_devices()
        self.get_logger().info('Available audio devices:')
        for i, d in enumerate(devices):
            self.get_logger().info(f'  [{i}] {d["name"]} — in:{d["max_input_channels"]} out:{d["max_output_channels"]}')

    def waypoint_callback(self, msg):
        self.current_waypoint = msg
        self.get_logger().info(f'At waypoint: {msg.current_waypoint_label}')
    
    # When the Robot reaches the Waypoint - Operator has to enter in some key information
    def trigger_callback(self, msg):

        if self.current_waypoint is None:
            self.get_logger().warn("No waypoint received yet")
            return

        if msg.data and not self.recording:
            self.get_logger().info("Waypoint reached — enter recording metadata")

            # room_directory = input("room-directory: ").strip()
            # excitation_path = input("excitation-path: ").strip()
            # occlusion_type = input("occlusion-type: ").strip()
            # occlusion_distance = input("occlusion-distance: ").strip()
            # occluded_type = input("occluded-type: ").strip()
            # occluded_distance = input("occluded-distance: ").strip()
            # base_filename = input("base-filename: ").strip()

            # self.room_directory = room_directory
            # self.excitation_path = excitation_path
            # self.occlusion_type = occlusion_type
            # self.occlusion_distance = occlusion_distance
            # self.occluded_type = occluded_type
            # self.occluded_distance = occluded_distance
            # self.base_filename = base_filename

            waypoint_label = self.current_waypoint.current_waypoint_label

            self.object_map = {
                "west_mid": {
                    "occlusion_type": "wood",
                    "occluded_type": "metal_ladder",
                    "occlusion_distance": "0.0m",
                    "occluded_distance": "0.5m"
                },
                "north_mid": {
                    "occlusion_type": "foam",
                    "occluded_type": "plastic_speaker",
                    "occlusion_distance": "0.0m",
                    "occluded_distance": "0.5m"
                },
                "east_mid": {
                    "occlusion_type": "wood",
                    "occluded_type": "foam_chair",
                    "occlusion_distance": "0.0m",
                    "occluded_distance": "0.5m"
                },
                "south_mid": {
                    "occlusion_type": "foam",
                    "occluded_type": "polyethylene_case",
                    "occlusion_distance": "0.0m",
                    "occluded_distance": "0.5m"
                }
            }

            if waypoint_label in self.object_map:
                metadata = self.object_map[waypoint_label]

                self.occlusion_type = metadata["occlusion_type"]
                self.occluded_type = metadata["occluded_type"]
                self.occlusion_distance = metadata["occlusion_distance"]
                self.occluded_distance = metadata["occluded_distance"]

                self.distance_dir = "0"

                self.base_filename = (
                    f"{waypoint_label}_"
                    f"{self.occlusion_type}_"
                    f"{self.occluded_type}"
                )

            else:
                self.get_logger().warn(f"No object mapping for {waypoint_label}")
                return

            self.distance_dir = (
                f"{self.occlusion_distance}-"
                f"{self.occluded_distance}"
            ).strip()

            self.record_directory = (
                f"{self.dataset_directory}/"
                f"{self.room_directory}/"
                f"{self.occlusion_type}/"
                f"{self.occluded_type}/"
                f"{self.distance_dir}"
            )

            Path(self.record_directory).mkdir(
                parents=True,
                exist_ok=True
            )
            
            self.load_excitation()

            thread = threading.Thread(target=self.run_recording_session)
            thread.daemon = True
            thread.start()

    def plot_ir(start_sample, end_sample, rir_cropped, i, save_dir):

        # make sure folder exists
        os.makedirs(save_dir, exist_ok=True)

        plt.figure(figsize=(10, 4))
        plt.plot(np.arange(start_sample, end_sample), rir_cropped)

        plt.title(f"IR Segment (Mic 1) - Samples {start_sample} to {end_sample} - Recording {i + 1}")
        plt.xlabel("Sample Index")
        plt.ylabel("Amplitude")
        plt.grid(True)
        plt.tight_layout()

        # 👉 SAVE FILE HERE
        filename = os.path.join(save_dir, f"ir_plot_recording_{i+1}.png")
        plt.savefig(filename, dpi=300)

        # optional: still show it
        plt.show()

        # important: free memory
        plt.close()

        print(f"Saved plot to: {filename}")

    def run_recording_session(self):
        if self.excitation is None:
            self.get_logger().error('No excitation loaded — skipping recording')
            return

        self.recording = True

        waypoint_label = 'unknown'
        waypoint_id    = 0

        if self.current_waypoint:
            waypoint_label = self.current_waypoint.current_waypoint_label
            waypoint_id    = self.current_waypoint.current_waypoint_id

        #session_dir = os.path.join(
        #    self.output_dir,
        #    f'waypoint_{waypoint_id}_{waypoint_label}',
        #    datetime.now().strftime('%Y%m%d_%H%M%S')
        #)

        session_dir = self.record_directory
        Path(session_dir).mkdir(parents=True, exist_ok=True)

        self.get_logger().info(f'Starting acoustic session — {self.repeat} repeats')
        self.get_logger().info(f'Saving to: {session_dir}')

        duration = len(self.excitation) / self.fs

        for i in range(self.repeat):

            self.get_logger().info(f'Recording {i+1}/{self.repeat} — playing {duration:.2f}s excitation')

            status_msg = String()

            status_msg.data = f'RECORDING {i+1}/{self.repeat} at {waypoint_label}'

            self.status_pub.publish(status_msg)

            try:

                # recorded = sd.playrec(
                #     self.excitation,
                #     samplerate=self.fs,
                #     channels=self.channels,
                #     device=(1, 13)
                # )

                # play excitation using system audio (Bluetooth-safe)
                tmp_wav = self.excitation_path

                subprocess.Popen(["aplay", tmp_wav, "-D", "default"])

                # record simultaneously
                recorded = sd.rec(
                    int(len(self.excitation)),
                    samplerate=self.fs,
                    channels=self.channels,
                    device=self.mic_device
                )

                # wait for playback duration
                time.sleep(len(self.excitation) / self.fs + 0.3)

                sd.wait()

            except Exception as e:
                self.get_logger().error(f'Audio error: {e}')
                break

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            # wav_path  = os.path.join(session_dir, f'recording_{i+1}_{timestamp}.wav')

            recorded_filename = (
                f"{timestamp}_"
                f"{self.base_filename}_"
                f"{i+1}.wav"
            )

            recorded_path = os.path.join(
                self.record_directory,
                recorded_filename
            )

            sf.write(recorded_path, recorded, self.fs)
            
            self.get_logger().info(f'Saved WAV: {recorded_path}')

            self.compute_and_save_ir(recorded, session_dir, i, timestamp)


            start_sample = 21300
            end_sample = 22000

            if i < self.repeat - 1:
                time.sleep(self.sleep_duration)

        self.recording = False
        self.get_logger().info('Acoustic session complete')

        complete_msg = Bool()
        complete_msg.data = True

        self.complete_pub.publish(complete_msg)

    def compute_and_save_ir(self, recorded, session_dir, repeat_index, timestamp):
        inv_filter = self.excitation[::-1]

        if recorded.ndim == 1:
            recorded = recorded[:, None]

        num_ch = recorded.shape[1]

        for ch in range(min(num_ch, self.channels)):
            ir_full = fftconvolve(recorded[:, ch], inv_filter, mode='full')
            N       = len(self.excitation)
            ir_full = ir_full[N:N * 2]
            ir      = ir_full[self.start_sample:self.end_sample]

            # ir_path   = os.path.join(session_dir, f'ir_repeat{repeat_index+1}_mic{ch+1}.npy')
                
            ir_filename = (
                f"{timestamp}_"
                f"{self.base_filename}_"
                f"ir_repeat{repeat_index+1}_mic{ch+1}.npy"
            )

            ir_path = os.path.join(
                self.record_directory,
                ir_filename
            )

            np.save(ir_path, ir)

        self.get_logger().info(f'IR saved for {self.channels} channels — repeat {repeat_index+1}')

def main(args=None):
    rclpy.init(args=args)
    node = AcousticRecorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
