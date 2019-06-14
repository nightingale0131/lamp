import imutils
import cv2

imfile = "../../maps/simple1.pgm"
im = cv2.imread(imfile)
gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) # img already gray, but convert to cv2 format
ret, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
cv2.imshow("sample", thresh)
cv2.waitKey(0)

cnts = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

for c in cnts:
    # M = cv2.moments(c)
    # cX = int(M["m10"]/M["m00"])
    # cY = int(M["m01"]/M["m00"])

    cv2.drawContours(im, [c], -1, (0,255,0),2)
    # cv2.circle(im, (cX, cY), 7, (255, 255, 255), -1)

    cv2.imshow("Image", im)
    cv2.waitKey(0)
