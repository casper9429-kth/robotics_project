import numpy as np
import cv2 # OpenCV
import random as rn
import rembg



# Function to take a picture from the camera
def find_object_morph(image):
    """
    Find object using morphological operations
    input: image (numpy array) in BGR format, directly from the camera
    return: ellipse (tuple) containing the center (x,y) and the major and minor axis lengths and angle of rotation of the ellipse 
    """
    img = image.copy()
    # Greyscale image
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Threshold image using adaptive thresholding
    #thresh = cv2.adaptiveThreshold(grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    thresh = cv2.adaptiveThreshold(grey, 255, cv2.BORDER_REPLICATE, cv2.THRESH_BINARY, 69, 5)
    
    # Morphological operation
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13,13))
    blob = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    blob = cv2.morphologyEx(blob, cv2.MORPH_DILATE, kernel)

    # Find contours in image 
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # sort contours by area
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # Fit ellipse to largest contour 
    #ellipse = cv2.fitEllipse(contours[0])
    #return ellipse


    new_img = np.ones_like(grey)*255
    
    for contour in contours:
        # draw contour on image
        cv2.drawContours(new_img, [contour], 0, (0,255,0), 2)
        # fit min enclosing ellipse to contour
        ellipsis = cv2.fitEllipse(contour)
        #check area of ellipse
        are_ellipse = np.pi*ellipsis[1][0]*ellipsis[1][1]
        if are_ellipse < img.shape[0]*img.shape[1]:
            cv2.ellipse(new_img, ellipsis, (0,255,0), 2)
        #circle = cv2.minEnclosingCircle(contour)
        #cv2.circle(new_img, (int(circle[0][0]), int(circle[0][1])), int(circle[1]), (0,255,0), 2)
        #square = cv2.minAreaRect(contour)        
        #box = cv2.boxPoints(square)
        
    
    new_img = 255-new_img
    # Inflate new image 
    kernel = np.ones((3,3),np.uint8)
    new_img = cv2.dilate(new_img,kernel,iterations = 1)
    
    # Find contours in new image
    contours, hierarchy = cv2.findContours(new_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    # Remove contours with area bigger than half of the image
    contours = [contour for contour in contours if cv2.contourArea(contour) < img.shape[0]*img.shape[1]/3]
        
    # Sort contours by area
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # draw contours on image
    #cv2.drawContours(img, contours, -1, (0,255,0), 2)

    # Get the largest contour
    largest_contour = contours[0]
    
    # Fit ellipse to largest contour
    ellipsis = cv2.fitEllipse(largest_contour)
    ##draw ellipse on image
    #cv2.ellipse(img, ellipsis, (0,255,0), 2)    
    ## Fit rectangle to largest contour
    #
    #
    ## plot new edges
    #cv2.imshow("new edges", new_img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    return ellipsis



def find_object_rembg(image):
    """
    Finds object using rembg library, returns ellipse fitted to object
    input: image (numpy array) in BGR format, directly from the camera
    return: ellipse (tuple) containing the center (x,y) and the major and minor axis lengths and angle of rotation of the ellipse 
    """
    # remove background from image
    img_without_background = rembg.remove(image)
    # Make img_without_background greyscale
    grey = cv2.cvtColor(img_without_background, cv2.COLOR_BGR2GRAY)
    # make grey image binary
    #grey = cv2.adaptiveThreshold(grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    grey[grey > 0] = 255
    
    # Find contours in image
    contours, hierarchy = cv2.findContours(grey, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # sort contours by area
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    contour = contours[0]
    
    # Fit ellipse,circle,rectangle to contour
    ellipsis = cv2.fitEllipse(contour)

    return ellipsis


def find_object_derivatives(image):
    """
    Finds object using derivatives
    input: image (numpy array) in BGR format, directly from the camera
    input: image (numpy array) in BGR format, directly from the camera
    return: ellipse (tuple) containing the center (x,y) and the major and minor axis lengths and angle of rotation of the ellipse 
    """
    # Read image using cv2.imread()
    #path = "data/sr_2.png"
    img = image.copy()
    # gaussian blur
    img = cv2.GaussianBlur(img, (7,7), 8)

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_b = img[:,:,0]
    img_g = img[:,:,1]
    img_r = img[:,:,2]
    img_s = img_hsv[:,:,1]
    img_v = img_hsv[:,:,2]

    # Calc the derivative in x and y direction for each img_gray, img_b, img_g, img_r, img_s, img_v
    img_gray = enhance(img_gray)
    img_b = enhance(img_b)
    img_g = enhance(img_g)
    img_r = enhance(img_r)
    img_s = enhance(img_s)
    img_v = enhance(img_v)

    img[:,:,0] = img_b
    img[:,:,1] = img_g
    img[:,:,2] = img_r
        
    # take greyscale image and apply thresholding
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    

    # convolution with gaussian filter
    img = cv2.GaussianBlur(img, (7,7), 1)    

    # find edges in image 
    edges = cv2.Canny(img, 58, 280) # 20, 300)

    # dilate edges to make them thicker
    #kernel = np.ones((10,10),np.uint8)
    #edges = cv2.dilate(edges,kernel,iterations = 1)

    # perform closing 
    kernel = np.ones((3,3),np.uint8)
    #edges = cv2.morphologyEx(edges, cv2.MORPH_OPEN, kernel)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    # find contours
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    new_img = np.ones_like(img)*255
    
    # sort contours by area
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    for contour in contours:
        # draw contour on image
        cv2.drawContours(img, [contour], 0, (0,255,0), 2)
        # fit min enclosing ellipse to contour
        ellipsis = cv2.fitEllipse(contour)
        #check area of ellipse
        are_ellipse = np.pi*ellipsis[1][0]*ellipsis[1][1]
        if are_ellipse < img.shape[0]*img.shape[1]/3:
            cv2.ellipse(new_img, ellipsis, (0,255,0), 2)
        circle = cv2.minEnclosingCircle(contour)
        cv2.circle(new_img, (int(circle[0][0]), int(circle[0][1])), int(circle[1]), (0,255,0), 2)
        #square = cv2.minAreaRect(contour)        
        #cv2.drawContours(new_img, [np.int0(cv2.boxPoints(square))], 0, (0,255,0), 2)  

        
    
    new_img = 255-new_img
    # Inflate new image 
    kernel = np.ones((3,3),np.uint8)
    new_img = cv2.dilate(new_img,kernel,iterations = 1)
    
    
    # Find contours in new image
    contours, hierarchy = cv2.findContours(new_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    # Remove contours with area bigger than half of the image
    contours = [contour for contour in contours if cv2.contourArea(contour) < img.shape[0]*img.shape[1]/3]
        
    # Sort contours by area
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    # draw contours on image
    #cv2.drawContours(img, contours, -1, (0,255,0), 2)

    # Get the largest contour
    largest_contour = contours[0]
    
    # Fit ellipse to largest contour
    ellipsis = cv2.fitEllipse(largest_contour)

    return ellipsis

def enhance(img):
    """
    This function enhances the image by calculating the gradient of the image and adding it to the original image with smart scaling
    """
    # Convert image to grayscale
    gray = img.copy()

    # Apply the Sobel operator in both the x and y directions
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)

    # Calculate the magnitude of the gradients
    magnitude = np.sqrt(sobel_x**2 + sobel_y**2)

    # Normalize the magnitude to the range [0, 255]
    magnitude = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)


    # Add the magnitude to the original image with smart scaling
    enhanced_img = cv2.addWeighted(img, 1, magnitude, 1, 0)

    return enhanced_img
