import argparse
import time

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.framework.formats import landmark_pb2
from pythonosc import udp_client
from pythonosc.osc_message_builder import OscMessageBuilder
import pyrealsense2 as rs

from utils import add_default_args

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
    add_default_args(parser)
    args = parser.parse_args()

    # create osc client
    client = udp_client.SimpleUDPClient(args.ip, args.port)

    # setup camera loop
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    pose = mp_pose.Pose(
        min_detection_confidence=args.min_detection_confidence, min_tracking_confidence=args.min_tracking_confidence)

    # create realsense pipeline
    pipeline = rs.pipeline()

    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

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
            cv2.imshow('MediaPipe OSC Pose', image)

            if cv2.waitKey(5) & 0xFF == 27:
                break
    finally:
        pose.close()
        pipeline.stop()


if __name__ == "__main__":
    main()
