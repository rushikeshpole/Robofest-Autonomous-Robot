import cv2 as cv

if __name__ == '__main__':
	img = cv.imread('autobot.jpg')
	cv.imwrite("autobot_new.jpg",img)	#新建文件autobot_new.jpg，并且把autobot.jpg写进去
	new_img = cv.imread('autobot_new.jpg') #读取新写入的图片
	while True :
		cv.imshow("frame",img)
		cv.imshow("new_frame",new_img)
		action = cv.waitKey(10) & 0xFF
		if action == ord('q') or action == 113:
			break
	img.release()
	cv.destroyAllWindows()
