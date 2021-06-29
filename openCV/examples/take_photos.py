# [Last update] [Moses C. Nah] 2021.06.29 
# [Code from Following]
# [REF] https://stackoverflow.com/questions/34588464/python-how-to-capture-image-from-webcam-on-click-using-opencv

import cv2
from datetime import date

cam = cv2.VideoCapture( 0 )         # Call the 1st listed webcam\
cv2.namedWindow( "take_photo" )     # Setting the title of the window (not always necessary)

img_cnt = 0

# [Backup] [Moses C. Nah] 2021.06.29
# In case if you want to save the date for the photos
#today   = date.today()
#date    = today.strftime("%d_%m_%H%M%S")

while True:
    ret, frame = cam.read( )        # cam.read( ) returns  
                                    # (1) return: True/False
                                    # (2)  frame: (width x height x 3) BGR value of the screen
    if not ret:                     # If the return isn't successful
        print("failed to grab frame")
        break

    cv2.imshow( "take_photo", frame )

    k = cv2.waitKey( 1 )            # Display (or wait) the image for 1ms and then move forward

    if k%256 == 27:                 # ESC (ASCII #27) pressed
        print( "Escape hit, closing..." )
        break

    elif k%256 == 32:               # SPACE (ASCII #32) pressed
        img_name = "pic_{}.png".format( img_cnt )
        cv2.imwrite( img_name, frame )
        print( "{} written!".format( img_name ) )
        img_cnt += 1

cam.release()
cv2.destroyAllWindows()
