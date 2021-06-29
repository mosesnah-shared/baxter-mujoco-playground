import cv2
import argparse
import pickle


parser = argparse.ArgumentParser()

parser.add_argument("-c", "--calibrated", help = "turn on calibration", action = 'store_true')
args   = parser.parse_args()

vid = cv2.VideoCapture(0)

if args.calibrated:
    f_name = "cam_vars.pkl"
    with open( f_name, 'rb' ) as file_object:
        raw_data = file_object.read( )

    raw_data = pickle.loads( raw_data ) # deserialization
    mtx, dist, optimal_camera_matrix, roi = raw_data

while( True ):

    # Capture the video frame-by-frame
    ret, frame = vid.read()

    if args.calibrated:
        # Undistort the image
        undistorted_image = cv2.undistort( frame, mtx, dist, None, optimal_camera_matrix )

        # Crop the image. Uncomment these two lines to remove black lines
        # on the edge of the undistorted image.
        x, y, w, h  = roi
        frame       = undistorted_image[y:y+h, x:x+w]                           # Rewriting the frame

    cv2.imshow('frame', frame)                                                  # Display the resulting frame

    k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input

                                            # [BACKUP] ord('q'):
    if k%256 == 27:                         # If (ESC) key is given, stop the video
        print( "ESC inputted, Close Camera!" )
        break

vid.release()
cv2.destroyAllWindows()
