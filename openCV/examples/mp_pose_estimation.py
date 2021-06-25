# Source code from following:
# [REF] https://google.github.io/mediapipe/solutions/pose.html

import cv2
import mediapipe as mp                                                          # Install as [REF] https://google.github.io/mediapipe/getting_started/python.html
                                                                                # Use pip3 AFTER you come into virtual environment
mp_drawing = mp.solutions.drawing_utils
mp_pose    = mp.solutions.pose

cam = cv2.VideoCapture( 0 )                                                     # For webcam input: getting the first list of the webcam

N = 0

with mp_pose.Pose( min_detection_confidence = 0.5,
                    min_tracking_confidence = 0.5 ) as pose:
  while cam.isOpened():
    success, image = cam.read()
    N = N + 1
    if not success:
      print( "Ignoring empty camera frame." )
      # If loading a video, use 'break' instead of 'continue'.
      continue



    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor( cv2.flip( image, 1 ), cv2.COLOR_BGR2RGB )
    image_height, image_width, _ = image.shape

    # [Moses C. Nah] [Backup] [2021.06.25]
    # print( "[Height] {H} [Width] {W}".format( H = image_height , W = image_width  ) )   # In case you want to know the size of the image.


    # To improve performance, optionally mark the image as not writeable to pass by reference.
    image.flags.writeable = False
    results               = pose.process( image )

    # Draw the pose annotation on the image.
    image.flags.writeable = True
    image                 = cv2.cvtColor( image, cv2.COLOR_RGB2BGR )
    mp_drawing.draw_landmarks( image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS )

    # [Comments] [Moses C. Nah] [2021.06.25]
    # [REF]
    # POSE_LANDMARKS
    # A list of pose landmarks. Each landmark consists of the following:
    # x and y   : Landmark coordinates normalized to [0.0, 1.0] by the image width and height respectively.
    # z         : Represents the landmark depth with the depth at the midpoint of hips being the origin, and the smaller the value the closer the landmark is to the camera. The magnitude of z uses roughly the same scale as x.
    # visibility: A value in [0.0, 1.0] indicating the likelihood of the landmark being visible (present and not occluded) in the image.

    print( "HI", results.pose_landmarks )   # the xyz positions of the landmarks (the info is in xyz, how can they know?)

    cv2.imshow( 'MediaPipe Pose', image )

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()                                                         # Destroy all the windows
