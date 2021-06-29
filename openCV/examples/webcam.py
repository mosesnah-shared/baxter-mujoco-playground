import cv2

vid = cv2.VideoCapture(0)

while( True ):

    # Capture the video frame-by-frame
    ret, frame = vid.read()

    # Display the resulting frame
    cv2.imshow('frame', frame)


    # Undistort the image
    #undistorted_image = cv2.undistort(distorted_image, mtx, dist, None,
    #                                    optimal_camera_matrix)

    k = cv2.waitKey( 1 )                    # Wait for 1ms and get the key input
    
    if k%256 == 27:                         # If (ESC) key is given, stop the video
        print( "ESC inputted, Close Camera!" )
        break

vid.release() 
cv2.destroyAllWindows()
