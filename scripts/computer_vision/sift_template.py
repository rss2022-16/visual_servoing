import cv2
import imutils
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
    """
    Helper function to print out images, for debugging.
    Press any key to continue.
    """
    winname = "Image"
    cv2.namedWindow(winname)        # Create a named window
    cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
    cv2.imshow(winname, img)
    cv2.waitKey()
    cv2.destroyAllWindows()

def cd_sift_ransac(img, template):
    """
    Implement the cone detection using SIFT + RANSAC algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    # Minimum number of matching features
    MIN_MATCH = 10
    # Create SIFT
    sift = cv2.xfeatures2d.SIFT_create()

    # Compute SIFT on template and test image
    kp1, des1 = sift.detectAndCompute(template,None)
    kp2, des2 = sift.detectAndCompute(img,None)

    # Find matches
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1,des2,k=2)

    # Find and store good matches
    good = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            good.append(m)

    # If enough good matches, find bounding box
    if len(good) > MIN_MATCH:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        # Create mask
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()

        h, w = template.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)


        ########## YOUR CODE STARTS HERE ##########

        transformed_pts = cv2.perspectiveTransform(pts, M)
        x_min = y_min = 100000
        x_max = y_max = 0
        for pt in transformed_pts:
            pt = pt[0]
            if pt[0] > x_max:
                x_max = pt[0]
            if pt[0] < x_min:
                x_min = pt[0]
            if pt[1] > y_max:
                y_max = pt[1]
            if pt[1] < y_min:
                y_min = pt[1]

        ########### YOUR CODE ENDS HERE ###########

        image_print(template)
        cv2.rectangle(img, (x_min, y_min),
                        (x_max, y_max), (120, 0, 120), 2)
        cv2.imshow("Visualize", img)
        cv2.waitKey(0)

        # Return bounding box

        # print x_max, y_max, x_min, y_min
        return ((x_min, y_min), (x_max, y_max))
    else:

        print "[SIFT] not enough matches; matches: ", len(good)

        # Return bounding box of area 0 if no match found
        return ((0,0), (0,0))

def cd_template_matching(img, template):
    """
    Implement the cone detection using template matching algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    template_canny = cv2.Canny(template, 50, 200)

    # Perform Canny Edge detection on test image
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_canny = cv2.Canny(grey_img, 50, 200)

    # Get dimensions of template
    (img_height, img_width) = img_canny.shape[:2]
    (tmp_height, tmp_width) = template_canny.shape[:2]

    # Keep track of best-fit match
    best_match = None

    # Loop over different scales of image
    for scale in np.linspace(1.5, .5, 50):
        # Resize the image
        resized_template = imutils.resize(template_canny, width = int(template_canny.shape[1] * scale))
        (h,w) = resized_template.shape[:2]
        # Check to see if test image is now smaller than template image
        if resized_template.shape[0] > img_height or resized_template.shape[1] > img_width:
            continue

        ########## YOUR CODE STARTS HERE ##########
        # Use OpenCV template matching functions to find the best match
        # across template scales.

        result = cv2.matchTemplate(img_canny, resized_template, cv2.TM_CCOEFF)
        (minVal, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

        if best_match is None or (maxVal > best_match[0] and minVal > best_match[1]):
            best_match = (maxVal, minVal, maxLoc, scale)

    print best_match
    # image_print(template)
    # image_print(grey_img)
    (_, _, maxLoc, scale) = best_match
    r = scale
    (startX, startY) = (int(maxLoc[0]), int(maxLoc[1]))
    (endX, endY) = (int(maxLoc[0] + (tmp_width * r)), int(maxLoc[1] + (tmp_height * r)))

    print ((startX, startY),(endX, endY))

    cv2.imshow("temp", template_canny)
    cv2.rectangle(img_canny, (startX, startY),
                    (endX, endY), (120, 0, 120), 2)
    cv2.imshow("Visualize", img_canny)
    cv2.waitKey(0)

        # Remember to resize the bounding box using the highest scoring scale
        # x1,y1 pixel will be accurate, but x2,y2 needs to be correctly scaled
    bounding_box = ((startX, startY),(endX, endY))
        ########### YOUR CODE ENDS HERE ###########

    return bounding_box
