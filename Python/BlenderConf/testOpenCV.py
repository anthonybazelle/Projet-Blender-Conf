import cv2.cv as cv

if __name__=='__main__':
    """
        Permet de suivre la couleur bleu, appui sur 'q' pour quitter
    """
    ma_camera = cv.CaptureFromCAM(0)
    cv.NamedWindow("Test_Webcam")
    while True:
        ma_frame = cv.QueryFrame(ma_camera)
        ma_frame2 = cv.CreateImage(cv.GetSize(ma_frame), 8, 3)
        ma_frame3 = cv.CreateImage(cv.GetSize(ma_frame2), 8, 1)
        cv.CvtColor(ma_frame, ma_frame2, cv.CV_BGR2HSV)
        #Recherche du bleu
        cv.InRangeS(ma_frame2,cv.Scalar(90, 0, 0), \
                cv.Scalar(130, 255, 255), ma_frame3)
        cv.ShowImage("Test_Webcam", ma_frame3)

        if (cv.WaitKey(10) % 0x100) == 113:
            break
