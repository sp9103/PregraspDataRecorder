#include <stdio.h>
#include <afx.h>
#include <stdlib.h>
#include <conio.h>

#include "KinectMangerThread.h"
#include "ColorBasedTracker.h"

#define DEFAULT_PATH "data"

void writeDepthData(cv::Mat src, char* path, char* name);
void CreateRGBDdir(const char* className);

int main(){
	KinectMangerThread kinectManager;
	ColorBasedTracker tracker;

	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2 + 40, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);

	//���丮 ����
	char dirName[256];
	kinectManager.Initialize(RobotROI);
	printf("Enter class or Obj Name : ");
	scanf("%s", dirName);
	CreateRGBDdir(dirName);

	printf("press any key if window created.\n");
	getch();
	cv::Mat backRGB = kinectManager.getImg();
	cv::Mat backDepth = kinectManager.getDepth();
	if(backRGB.channels() == 4)	cv::cvtColor(backRGB, backRGB, CV_BGRA2BGR);
	cv::imshow("background", backRGB);
	cv::waitKey(1);

	//LOOP
	bool isSaved = false;
	std::vector<cv::Mat> ImgVec, DepthVec, PCVec;
	while(1){
		//������ �Էº�
		cv::Mat kinectImg = kinectManager.getImg();
		cv::Mat KinectDepth = kinectManager.getDepth();
		cv::Mat kinectPC = kinectManager.getPointCloud();

		//�����
		if(isSaved){
			ImgVec.push_back(kinectImg.clone());
			DepthVec.push_back(KinectDepth.clone());
			PCVec.push_back(kinectPC.clone());
		}
		//ó����
		else if(!isSaved && ImgVec.size() != 0 && DepthVec.size() != 0 && PCVec.size() != 0){
			//TO-DO

			ImgVec.clear();
			DepthVec.clear();
			PCVec.clear();
		}

		char keyInput = cv::waitKey(10);
		if(keyInput == 'q' || keyInput == 27)	break;
		else if(keyInput = 's')	isSaved = !isSaved;

		cv::imshow("kinectImg", kinectImg);
	}

	kinectManager.Deinitialize();

	return 0;
}

void writeDepthData(cv::Mat src, char* path, char* name){
	//Depth Infomation write
	char buf[256];
	sprintf(buf, "%s\\%s.bin", path, name);
	FILE *fp = fopen(buf, "wb");
	fwrite(&src.rows, sizeof(int), 1, fp);
	fwrite(&src.cols, sizeof(int), 1, fp);
	int Type = src.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < src.rows * src.cols; i++)		fwrite(&src.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);
}

void CreateRGBDdir(const char* className){
	TCHAR szDir[MAX_PATH] = {0,};
	TCHAR RGBDDir[MAX_PATH] = {0,};
	TCHAR DepthDir[MAX_PATH] = {0,};
	TCHAR xyzDir[MAX_PATH] = {0,};
	TCHAR procDepthDir[MAX_PATH] = {0, };
	char dirpath[256];
	sprintf(dirpath, "%s\\%s\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), szDir, MAX_PATH);
	bool mkdir_check = CreateDirectory(szDir, NULL);									//��Ʈ ���丮
	sprintf(dirpath, "%s\\%s\\RGB\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), RGBDDir, MAX_PATH);
	mkdir_check = CreateDirectory(RGBDDir, NULL);											//�÷� ���丮 - ����
	sprintf(dirpath, "%s\\%s\\ANGLE\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//Angle
	sprintf(dirpath, "%s\\%s\\DEPTHMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), DepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(DepthDir, NULL);											//���� ���丮 - ����
	sprintf(dirpath, "%s\\%s\\XYZMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//����Ʈ Ŭ���� ���丮 - ����
	sprintf(dirpath, "%s\\%s\\BACKGROUND\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);
	sprintf(dirpath, "%s\\%s\\PROCESSIMG\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);
	sprintf(dirpath, "%s\\%s\\PROCDEPTH\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), procDepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(procDepthDir, NULL);
}