# RealSense Pose Detector
A simple pose detector which runs with the realsense framework.

### Install & Run

Currently this is only tested on Windows and macOS. It's recommended to use Python3 (`>3.7`) and a virtual environment.

```bash
python install -r requirements.txt
```

To run an example use the basic python command to start up the script.

```bash
# start pose detection with realsense camera
python pose.py
```

### Full-Body Pose Landmark Model (BlazePose Tracker)
The landmark model currently included in MediaPipe Pose predicts the location of 33 full-body landmarks (see figure below), each with (`x, y, z, visibility`). Note that the z value should be discarded as the model is currently not fully trained to predict depth, but this is something we have on the roadmap.

![Pose Description](readme/pose_tracking_full_body_landmarks.png)

*[Reference: mediapipe/solutions/pose](https://google.github.io/mediapipe/solutions/pose#pose-landmark-model-blazepose-tracker)*

#### Format

- `count` - Indicates how many poses are detected (currently only `0` or `1`)
- list of landmarks (`33` per pose) (if pose has been detected)
    - `x` - X-Position of the landmark
    - `y` - Y-Position of the landmark
    - `z` - Z-Position of the landmark
    - `visibility` - Visibility of the landmark

```
/mediapipe/pose [count, x, y, z, visibility, x, y, z, visibility ...]
```

### Help

```
usage: pose.py [-h] [--resolution width height] [--fps FPS]
               [--model-complexity MODEL_COMPLEXITY] [--no-smooth-landmarks]
               [--static-image-mode] [-mdc MIN_DETECTION_CONFIDENCE]
               [-mtc MIN_TRACKING_CONFIDENCE] [--ip IP] [--port PORT]

optional arguments:
  -h, --help            show this help message and exit

RealSense:
  --resolution width height
                        Resolution of the realsense stream.
  --fps FPS             Framerate of the realsense stream.

MediaPipe:
  --model-complexity MODEL_COMPLEXITY
                        Set model complexity (0=Light, 1=Full, 2=Heavy).
  --no-smooth-landmarks
                        Disable landmark smoothing.
  --static-image-mode   Enables static image mode.
  -mdc MIN_DETECTION_CONFIDENCE, --min-detection-confidence MIN_DETECTION_CONFIDENCE
                        Minimum confidence value ([0.0, 1.0]) for the
                        detection to be considered successful.
  -mtc MIN_TRACKING_CONFIDENCE, --min-tracking-confidence MIN_TRACKING_CONFIDENCE
                        Minimum confidence value ([0.0, 1.0]) to be considered
                        tracked successfully.

Network:
  --ip IP               The ip of the OSC server
  --port PORT           The port the OSC server is listening on
```

### About
MIT License - Copyright (c) 2021 Florian Bruggisser

Based on the ideas of [mediapipe-osc](https://github.com/cansik/mediapipe-osc).