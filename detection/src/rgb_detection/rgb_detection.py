import numpy as np
import cv2 # OpenCV


def rgb_detection(image):
    """
    Input: rgb image as numpy array
    Output: ellipse as tuple ((center_columns,center_row), (width, height), angle)
    
    Peforms canny edge detection on each channel of the image and greyscale, 
    and then finds the contours of the combined edges. 
    After that an ellipse is fitted to the outer contour of the combined contours.
    """
    # Load the image
    #file_path = 'data/ani_3_2.jpg'
    #image = cv2.imread(filename=file_path) # B G R

    # Gaussian blur the image with a kernel size of 5 and a standard deviation of 10
    image = cv2.GaussianBlur(src=image, ksize=(5, 5), sigmaX=10, sigmaY=10)

    # Downsample the image by a factor of 10
    image = cv2.resize(src=image, dsize=(0, 0), fx=0.1, fy=0.1)




    # Peform canny edge detection for each channel of the image
    edges_b = cv2.Canny(image=image[:, :, 0], threshold1=200, threshold2=700)
    edges_g = cv2.Canny(image=image[:, :, 1], threshold1=200, threshold2=700)
    edges_r = cv2.Canny(image=image[:, :, 2], threshold1=200, threshold2=700)
    edges_gray = cv2.Canny(image=cv2.cvtColor(src=image, code=cv2.COLOR_BGR2GRAY), threshold1=200, threshold2=700)

    # Find contours
    #contours_b, hierarchy_b = cv2.findContours(image=edges_b, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    #contours_g, hierarchy_g = cv2.findContours(image=edges_g, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    #contours_r, hierarchy_r = cv2.findContours(image=edges_r, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    #contours_gray, hierarchy_gray = cv2.findContours(image=edges_gray, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

    contours_b, hierarchy_b = cv2.findContours(image=edges_b,mode=cv2.RETR_EXTERNAL,method=cv2.CHAIN_APPROX_NONE)
    contours_g, hierarchy_g = cv2.findContours(image=edges_g,mode=cv2.RETR_EXTERNAL,method=cv2.CHAIN_APPROX_NONE)
    contours_r, hierarchy_r = cv2.findContours(image=edges_r,mode=cv2.RETR_EXTERNAL,method=cv2.CHAIN_APPROX_NONE)
    contours_gray, hierarchy_gray = cv2.findContours(image=edges_gray,mode=cv2.RETR_EXTERNAL,method=cv2.CHAIN_APPROX_NONE)


    # Merge all contours into one contours, and draw it
    # Merge all contours

    contours = contours_b + contours_g + contours_r + contours_gray

    # Transform the boundaries of the contours to one contour
            
    # Assume 'contours' is the list of contours obtained from your previous code

    # Create a blank image to draw the contours
    blank_image = np.zeros_like(image)

    # Draw all the contours on the blank image
    for contour in contours:
        cv2.drawContours(image=blank_image, contours=[contour], contourIdx=-1, color=(255, 255, 255), thickness=10)



    # Find the outer contour of the combined contours
    contour_of_contours, _ = cv2.findContours(image=cv2.cvtColor(src=blank_image, code=cv2.COLOR_BGR2GRAY), mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)

    # Fit ellipses to the outer contour of the combined contours
    ellipsis = cv2.fitEllipse(contour_of_contours[0])
    
    
    # cv2.ellipse(image, ellipsis, (0, 0, 255), 2)


    # ## Draw the contour of the contours on the original image (optional)
    # result_image = image.copy()
    # cv2.drawContours(image=result_image, contours=contour_of_contours, contourIdx=-1, color=(0, 255, 0), thickness=3)

    # # Display the result image with the contour of the contours (optional)
    # cv2.imshow('Result Image with Contour of Contours', result_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    


    return ellipsis