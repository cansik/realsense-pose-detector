import argparse
import time

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.framework.formats import landmark_pb2
from pythonosc import udp_client
from pythonosc.osc_message_builder import OscMessageBuilder
import pyrealsense2 as rs

OSC_ADDRESS = "/mediapipe/pose"


def send_pose(client: udp_client,
              landmark_list: landmark_pb2.NormalizedLandmarkList):
    if landmark_list is None:
        client.send_message(OSC_ADDRESS, 0)
        return

    # create message and send
    builder = OscMessageBuilder(address=OSC_ADDRESS)
    builder.add_arg(1)
    for landmark in landmark_list.landmark:
        builder.add_arg(landmark.x)
        builder.add_arg(landmark.y)
        builder.add_arg(landmark.z)
        builder.add_arg(landmark.visibility)
    msg = builder.build()
    client.send(msg)


def main():
    # read arguments
    parser = argparse.ArgumentParser()
    rs_group = parser.add_argument_group("RealSense")
    rs_group.add_argument("--resolution", default=[640, 480], type=int, nargs=2, metavar=('width', 'height'),
                          help="Resolution of the realsense stream.")
    rs_group.add_argument("--fps", default=30, type=int,
                          help="Framerate of the realsense stream.")

    mp_group = parser.add_argument_group("MediaPipe")
    mp_group.add_argument("--model-complexity", default=1, type=int,
                          help="Set model complexity (0=Light, 1=Full, 2=Heavy).")
    mp_group.add_argument("--no-smooth-landmarks", action="store_false", help="Disable landmark smoothing.")
    mp_group.add_argument("--static-image-mode", action="store_true", help="Enables static image mode.")
    mp_group.add_argument("-mdc", "--min-detection-confidence", type=float, default=0.5,
                          help="Minimum confidence value ([0.0, 1.0]) for the detection to be considered successful.")
    mp_group.add_argument("-mtc", "--min-tracking-confidence", type=float, default=0.5,
                          help=" Minimum confidence value ([0.0, 1.0]) to be considered tracked successfully.")

    nw_group = parser.add_argument_group("Network")
    nw_group.add_argument("--ip", default="127.0.0.1",
                          help="The ip of the OSC server")
    nw_group.add_argument("--port", type=int, default=7400,
                          help="The port the OSC server is listening on")

    args = parser.parse_args()

    # create osc client
    client = udp_client.SimpleUDPClient(args.ip, args.port)

    # setup camera loop
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    pose = mp_pose.Pose(
        smooth_landmarks=args.no_smooth_landmarks,
        static_image_mode=args.static_image_mode,
        model_complexity=args.model_complexity,
        min_detection_confidence=args.min_detection_confidence,
        min_tracking_confidence=args.min_tracking_confidence)

    # create realsense pipeline
    pipeline = rs.pipeline()

    width, height = args.resolution

    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

    profile = pipeline.start(config)

    prev_frame_time = 0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                break

            image = np.asanyarray(color_frame.get_data())

            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            results = pose.process(image)

            # send the pose over osc
            send_pose(client, results.pose_landmarks)

            # Draw the pose annotation on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            current_time = time.time()
            fps = 1 / (current_time - prev_frame_time)
            prev_frame_time = current_time

            cv2.putText(image, "FPS: %.0f" % fps, (7, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow('RealSense Pose Detector', image)

            if cv2.waitKey(5) & 0xFF == 27:
                break
    finally:
        pose.close()
        pipeline.stop()


if __name__ == "__main__":
    main()
