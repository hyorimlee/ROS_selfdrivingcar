#!/usr/bin/env python
# coding: utf-8

# In[1]:


import cv2
import numpy as np


# In[2]:


def canny(image):
    #make it have 1 channel(gray)
    gray = cv2.cvtColor(lane_image,cv2.COLOR_RGB2GRAY)

    #blur it(gaussian)
    blur = cv2.GaussianBlur(gray,(5,5),0)

    #image, low threshold, high thresthold
    canny = cv2.Canny(blur, 50,150)
    return canny


# In[3]:


#load image
image = cv2.imread('test.jpg')
lane_image = np.copy(image)

canny=canny(lane_image)

#show it
cv2.imshow('result',canny)


# In[4]:


cv2.waitKey(0)


# In[ ]:




