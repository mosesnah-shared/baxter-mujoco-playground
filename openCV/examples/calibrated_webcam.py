import cv2
import pickle


# define a video capture object
vid = cv2.VideoCapture(0)

with open( "cam_vars.pkl",'rb') as file_object:
    raw_data = file_object.read()

raw_data = pickle.loads( raw_data ) # deserialization

mtx, dist, optimal_camera_matrix = raw_data

while( True ):

    # Capture the video frame
    # by frame
    ret, frame = vid.read()

    # Undistort the image
    undistorted_image = cv2.undistort( frame, mtx, dist, None, optimal_camera_matrix )

    # Display the resulting frame
    cv2.imshow('frame', undistorted_image)


    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
