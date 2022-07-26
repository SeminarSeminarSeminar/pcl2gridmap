import cv2
import numpy as np
from matplotlib import pyplot as plt

path = '/home/alpha/Pictures/ORI_RE.png'

filename = path
img = cv2.imread(filename)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
corners = cv2.goodFeaturesToTrack(gray,500, 0.01, 1)
corners = np.int0(corners)
for i in corners:
    x,y = i.ravel()
    cv2.circle(img,(x,y),5,255,-1)
plt.imshow(img),plt.show()