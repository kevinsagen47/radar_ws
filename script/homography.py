#!/usr/bin/env python3
 
import cv2
import numpy as np
 
if __name__ == '__main__' :
    
    pts_src = np.array([
                [0.8630979061*500,640-0.4199261367*500, 1],
                [0.9365850091*500,640-0.2099630684*500, 1],
                [0.9121673107*500,640+0.08589398116*500, 1],
                [0.9013402462*500,640+0.3299419582*500, 1],
                [0.9823723435*500,640+0.5671729445*500,1]])
    pts_src=np.around(pts_src,0)
    print(pts_src)
    pts_dst = np.array([[1103, 265, 1], [875, 282,1], [583, 291,1],[339, 303,1],[124,316,1]])
    print("Process Noise Covariance Matrix Q")
    kf=np.array([["Ex",0,0,0,0,0],
                [0,"Ey",0,0,0,0],
                [0,0,"Ev_x",0,0,0],
                [0,0,0,1,"Ev_y",0],
                [0,0,0,0,1,"Ew"],
                [0,0,0,0,0,"Eh"]])
    print(kf)
    #pts_src = np.array([[141, 131,1], [480, 159,1], [493, 630,1],[64, 601,1]])
    #pts_dst = np.array([[318, 256],[534, 372],[316, 670],[73, 473]])

    # Calculate Homography
    #h, status = cv2.findHomography(pts_dst,pts_src)
    h, status = cv2.findHomography(pts_src, pts_dst)
    print(h)
    print (pts_src[0])
    hasil=h@pts_src[0]
    print(hasil)
    '''
    # Warp source image to destination based on homography
    im_out = cv2.warpPerspective(im_src, h, (im_dst.shape[1],im_dst.shape[0]))
 
    # Display images
    cv2.imshow("Source Image", im_src)
    cv2.imshow("Destination Image", im_dst)
    cv2.imshow("Warped Source Image", im_out)
 
    cv2.waitKey(0)
    '''