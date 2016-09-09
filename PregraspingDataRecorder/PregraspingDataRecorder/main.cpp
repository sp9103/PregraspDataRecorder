#include <stdio.h>
#include <afx.h>
#include <stdlib.h>
#include <conio.h>
#include <random>

#include "KinectMangerThread.h"
#include "ColorBasedTracker.h"

#define DEFAULT_PATH "data"
#define SAMPLING_COUNT 80

void writeDepthData(cv::Mat src, char* path, char* name);
void CreateRGBDdir(const char* className);
bool writeData(cv::Mat RGBimg, cv::Mat DEPTHimg, cv::Mat pointCloud, char* path, const int count, cv::Mat backRGB, cv::Mat backDepth);


int main(){
	KinectMangerThread kinectManager;

	cv::Rect RobotROI((KINECT_DEPTH_WIDTH - 160) / 2 + 40, (KINECT_DEPTH_HEIGHT- 160) / 2, 160, 160);

	//디렉토리 생성
	char dirName[256];
	kinectManager.Initialize(RobotROI);
	printf("Enter class or Obj Name : ");
	scanf("%s", dirName);
	CreateRGBDdir(dirName);

	printf("press any key if window created & save backGround.\n");
	getch();
	cv::Mat backRGB = kinectManager.getImg();
	cv::Mat backDepth = kinectManager.getDepth();
	if(backRGB.channels() == 4)	cv::cvtColor(backRGB, backRGB, CV_BGRA2BGR);
	cv::imshow("background", backRGB);
	cv::waitKey(1);
	char buf[256];
	strcpy(buf, DEFAULT_PATH);
	strcat(buf, dirName);
	sprintf(buf, "%s\\%s", DEFAULT_PATH, dirName);
	writeDepthData(backDepth, buf, "backDepth");
	strcat(buf, "\\backRGB.bmp");
	cv::imwrite(buf, backRGB);

	//LOOP
	bool isSaved = false;
	int count = 0;
	std::vector<cv::Mat> ImgVec, DepthVec, PCVec/*, ProcVec*/;
	ColorBasedTracker tracker;
	tracker.InsertBackGround(backRGB, backDepth);
	//std::vector<std::vector<cv::Rect>> blobVec;
	while(1){
		//프레임 입력부
		int tick = GetTickCount();
		cv::Mat kinectImg = kinectManager.getImg();
		cv::Mat KinectDepth = kinectManager.getDepth();
		cv::Mat kinectPC = kinectManager.getPointCloud();

		//저장부
		if(isSaved){
			//std::vector<cv::Rect> tempVec;
			/*armSub = tracker.DeleteArm(kinectImg, KinectDepth);
			if(armSub.rows > 0){
				procImg = tracker.calcBlobAll(armSub, KinectDepth, &tempVec);
			}
			else
				continue;

			if(armSub.rows <= 0 || procImg.rows <= 0)
				continue;*/
			printf("saved [%d]\n", ImgVec.size());
			ImgVec.push_back(kinectImg.clone());
			DepthVec.push_back(KinectDepth.clone());
			PCVec.push_back(kinectPC.clone());
			//ProcVec.push_back(procImg.clone());
			//blobVec.push_back(tempVec);
			//cv::imshow("procImg", procImg);
		}
		//처리부
		else if(!isSaved && ImgVec.size() != 0 && DepthVec.size() != 0 && PCVec.size() != 0){
			//TO-DO
			printf("Sampling & store\n");
			char tempIdxBuf[256];
			sprintf(tempIdxBuf, "%s\\%s\\IdxSet.txt", DEFAULT_PATH, dirName);
			FILE *fp;
			if(count == 0)	fp = fopen(tempIdxBuf, "w");
			else			fp = fopen(tempIdxBuf, "a");

			//store
			int startIdx = count;
			std::vector<int> indexBox;
			for(int i = 0; i < ImgVec.size(); i++){
				bool writeCheck = writeData(ImgVec.at(i), DepthVec.at(i), PCVec.at(i), dirName, count, backRGB, backDepth);
				if(writeCheck){
					indexBox.push_back(count);
					count++;
				}
			}
			int endIdx = count - 1;
			//sampling
			int firstEND = (indexBox.size() - 1) * 0.2f;
			int secondEnd = (indexBox.size() - 1) * 0.4f;
			int ThirdEnd = (indexBox.size() - 1);
			std::uniform_int_distribution<int>  firstSampler(0, firstEND);
			std::uniform_int_distribution<int>  seconstSampler(firstEND+1, secondEnd);
			std::uniform_int_distribution<int>	thirdSampler(secondEnd+1, ThirdEnd);
			std::random_device                  rand_dev;
			std::mt19937                        generator(rand_dev());
			for(int i = 0; i < SAMPLING_COUNT; i++){
				int firstIdx = indexBox.at(firstSampler(generator));
				int secondIdx = indexBox.at(seconstSampler(generator));
				int ThiredIdx = indexBox.at(thirdSampler(generator));
				int goalIdx = indexBox.at(indexBox.size() - 1);
				fprintf(fp, "%d %d\n", firstIdx, secondIdx);
				fprintf(fp, "%d %d\n", secondIdx, ThiredIdx);
				fprintf(fp, "%d %d\n", ThiredIdx, goalIdx);
			}

			indexBox.clear();
			fclose(fp);
			ImgVec.clear();
			DepthVec.clear();
			PCVec.clear();
			//blobVec.clear();
			//ProcVec.clear();

			printf("==================process complete!=============================\n");
		}

		char keyInput = cv::waitKey(33);
		if(keyInput == 'q' || keyInput == 27)	break;
		else if(keyInput == 's')	isSaved = !isSaved;

		char buf[256];
		tick = GetTickCount() - tick;
		sprintf(buf, "%.2f fps", 1000.f/ (float)tick);
		cv::Mat tempImg = kinectImg.clone();
		cv::putText(tempImg, buf, cv::Point(0,150), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(0,255,255));
		if(isSaved)	cv::rectangle(tempImg, cv::Rect(0,0, 160,160), cv::Scalar(0,0,255), 2);
		cv::imshow("kinectImg", tempImg);
		tempImg.release();
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
	bool mkdir_check = CreateDirectory(szDir, NULL);									//루트 디렉토리
	sprintf(dirpath, "%s\\%s\\RGB\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), RGBDDir, MAX_PATH);
	mkdir_check = CreateDirectory(RGBDDir, NULL);											//컬러 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\ANGLE\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//Angle
	sprintf(dirpath, "%s\\%s\\DEPTHMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), DepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(DepthDir, NULL);											//뎁스 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\XYZMAP\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);											//포인트 클라우드 디렉토리 - 원본
	sprintf(dirpath, "%s\\%s\\PROCESSIMG\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), xyzDir, MAX_PATH);
	mkdir_check = CreateDirectory(xyzDir, NULL);
	sprintf(dirpath, "%s\\%s\\PROCDEPTH\0", DEFAULT_PATH, className);
	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, dirpath, strlen(dirpath), procDepthDir, MAX_PATH);
	mkdir_check = CreateDirectory(procDepthDir, NULL);
}

bool writeData(cv::Mat RGBimg, cv::Mat DEPTHimg, cv::Mat pointCloud, char* path, const int count, cv::Mat backRGB, cv::Mat backDepth){
	if(RGBimg.channels() == 4)	cv::cvtColor(RGBimg, RGBimg, CV_BGRA2BGR);
	if(backRGB.channels() == 4)	cv::cvtColor(backRGB, backRGB, CV_BGRA2BGR);

	char pathBuf[256], buf[256], id[256];
	sprintf(pathBuf, "%s\\%s", DEFAULT_PATH, path);
	itoa(count, id, 10);

	//store RGB
	sprintf(buf, "%s\\RGB\\%d.bmp", pathBuf, count);
	cv::imwrite(buf, RGBimg);
	//store Depth
	sprintf(buf, "%s\\DEPTHMAP", pathBuf);
	writeDepthData(DEPTHimg, buf, id);
	//store point cloud
	sprintf(buf, "%s\\XYZMAP\\%d.bin", pathBuf, count);
	FILE *fp = fopen(buf, "wb");
	fwrite(&pointCloud.rows, sizeof(int), 1, fp);
	fwrite(&pointCloud.cols, sizeof(int), 1, fp);
	int Type = pointCloud.type();
	fwrite(&Type, sizeof(int), 1, fp);
	for(int i = 0; i < pointCloud.rows * pointCloud.cols; i++)
		for(int c = 0; c < pointCloud.channels(); c++)
			fwrite(&pointCloud.at<cv::Vec3f>(i)[c], sizeof(float), 1, fp);
	fclose(fp);

	return true;
}