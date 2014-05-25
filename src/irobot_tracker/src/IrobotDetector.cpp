#include "IrobotDetector.h"

IrobotDetector::IrobotDetector()
{
	tempFlag=true;
	// ����ͷ�궨����
	camMatrix=(Mat_<float>(3,3)<<
		779.2414/2,0,319.5/2,
		0,779.2414/2,239.5/2,
		0,0,1);
	distCoeff = (Mat_<float>(5,1) << 0.26,-2.24,0,0,6.10);

	m_markerSize = Size(200, 100);
	// Ĭ��marker��sizeΪ200*100,markercorner��2d�ռ�Ϊ200*100�ľ���
	marker2dPt.push_back(Point2f(0, 0));
	marker2dPt.push_back(Point2f(m_markerSize.width-1, 0));
	marker2dPt.push_back(Point2f(m_markerSize.width-1, m_markerSize.height-1));
	marker2dPt.push_back(Point2f(0, m_markerSize.height-1));

	// 3d corner��������Ϊ��marker����Ϊԭ��
	
	marker3dPt.push_back(cv::Point3f(-119.0f,-85.0f,0));
	marker3dPt.push_back(cv::Point3f(+119.0f,-85.0f,0));
	marker3dPt.push_back(cv::Point3f(+119.0f,+85.0f,0));
	marker3dPt.push_back(cv::Point3f(-119.0f,+85.0f,0));

	// ����ɫ͸��ӳ��ʱ��
	markCorners2d.push_back(Point2f(0, 0));
	markCorners2d.push_back(Point2f(m_markerSize.width-1, 0));
	markCorners2d.push_back(Point2f(m_markerSize.width-1, m_markerSize.height-1));
	markCorners2d.push_back(Point2f(0, m_markerSize.height-1));

	findImageContours();
}

IrobotDetector::~IrobotDetector()
{}

void IrobotDetector::findImageContours()
{
	imC=imread(IMAGE_PATH);
	Mat imC2;
	cvtColor( imC, imC2, CV_BGR2GRAY );
	threshold(imC2,imCbw,255,255,CV_THRESH_OTSU|CV_THRESH_BINARY);
	findContours(imCbw,contoursO,hierarchyO,RETR_TREE, CHAIN_APPROX_SIMPLE);
	//findContours(imbw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	for(int i=0;i<contoursO.size();i++)
	{
		drawContours(imC,contoursO,i,Scalar(0,0,255),2);
		stringstream ss; 
		ss<<i;
		setLabel(imC,ss.str(),contoursO[i]);
		//matchShapes(contours[i],contoursO,CV_CONTOURS_MATCH_I1,0);
	}
	
	contoursOO=contoursO[24];


	vector<cv::Point> approx;
	approxPolyDP(contoursOO,approx,arcLength(contoursOO,true)*0.02,true);

	int i=8;
	while(i)matchContour.push_back(approx[--i]);

	drawGoodpt(imC,matchContour,Scalar(255,0,0));
}

void IrobotDetector::processFrame(Mat frame)
{
	contours.clear();
	hierarchy.clear();
	harrisCorners.clear();
	imcontour=frame.clone();
	cvtColor( frame, imgray, CV_BGR2GRAY );
	imgrayPreciseCorners=imgray.clone();
	imgrayTemplate=imgray.clone();
	findIrobot();
}

void IrobotDetector::findIrobot()
{
	isTracking=false;

	ContourBasedDetector();
	guess=false;
	if(isTracking)
	{
		drawGoodpt(imcontour,goodKeyPt,Scalar(255,0,0));//blue
		goodKeyPtTime=0;
		tempFlag=true;
	}
	else
	{
		BinBasedDetector();
		if(isTracking)
		{
			drawGoodpt(imcontour,goodKeyPt,Scalar(0,255,0));//green
			goodKeyPtTime=0;
			tempFlag=true;
		}
		else
		{
			//HarrisBasedDetector();
			//guess=true;
			if(isTracking)
			{
				drawGoodpt(imcontour,goodKeyPt,Scalar(0,0,255));//red
				tempFlag=true;
			}
			else
			{
				//MatchTemplateBaseDetector();
				//guess=true;
				if(isTracking)
				{
					drawGoodpt(imcontour,goodKeyPt,Scalar(0,0,0));
				}
			}
		}
	}

	if(isTracking)
	{
		marker2dPt.clear();
		marker2dPt.push_back(goodKeyPt[7]);
		marker2dPt.push_back(goodKeyPt[2]);
		marker2dPt.push_back(goodKeyPt[1]);
		marker2dPt.push_back(goodKeyPt[0]);

		//subpixel���marker corner�ľ���
		cornerSubPix(imgrayPreciseCorners, 
			marker2dPt,
			Size(5,5), 
			Size(-1,-1), 
			cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

		estimatePosition(marker3dPt,marker2dPt);
		drawPlain(imcontour);
	}

#if(DEBUG_ON)

	//imshow("match",imC);
	//imshow("contour",imcontour);
	//imshow("bw",imbw);
	//imshow("canny",imEdge);
	imshow("contour",imcontour);
	waitKey(1);

#endif

}

void IrobotDetector::ContourBasedDetector()
{
	matchMax=100;
	goodContour=-1;
	matchN=10;
	blur( imgray, imgray, Size(3,3) );
	Canny(imgray,imEdge,0,100,3);
	threshold(imgray,imbw,255,255,CV_THRESH_OTSU|CV_THRESH_BINARY);
	findContours(imEdge.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	for(int i=0;i<contours.size();i++)
	{
			
		//area detect
		if(contourArea(contours[i])<(2000/SCALE/SCALE)||contourArea(contours[i])>(120000/SCALE/SCALE)) continue;
		//drawContours(imcontour,contours,i,Scalar(255,0,0),2);
		//approx ploy
		vector<cv::Point> approx;
		approxPolyDP(contours[i],approx,arcLength(contours[i],true)*0.02,true);
		if(approx.size()!=8) continue;
		//cout<<"8 points pass"<<endl;

		// Convex?
		if (isContourConvex(approx)) continue;
		//cout<<"!isContourConvex pass"<<endl;

		// find the max 
		float maxDist = 0;
		int markerOrigin;
		for (int i=0; i<8; i++)
		{
			Point vecTemp = approx[i] - approx[(i+1)%8];
			float distSquared = vecTemp.dot(vecTemp);
			if(distSquared>maxDist)
			{
				markerOrigin=i;
				maxDist=distSquared;
			}
		}
		//maxDist
		if(maxDist<(3000/SCALE))continue;
			
		//cout<<"maxDist pass"<<endl;
		//right order of point
		rotate(approx.begin(), approx.begin() + markerOrigin, approx.end());

		/*
		for(int j=0;j<approx.size();j++)
		{
			Scalar colorSc(0,255,0);
			circle(imcontour,Point(approx[j].x,approx[j].y),5,colorSc,2);
			stringstream ss2;
			ss2<<j;
			putText(imcontour, ss2.str(), Point(approx[j].x+10,approx[j].y+10), cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255), 2, 8);
		}
		*/
		// cos=90
		//double cos=angle(approx[0], approx[2],approx[1]);
		//cout<<"cos: "<<cos<<endl;
		//if(cos>0.5||cos<-0.2) continue;


		//perspectiveTransform
		Mat canonicalImg;
		vector <Point2f> makerDetect;
		makerDetect.push_back(approx[2]);
		makerDetect.push_back(approx[7]);
		makerDetect.push_back(approx[0]);
		makerDetect.push_back(approx[1]);
		Mat M = getPerspectiveTransform(makerDetect, markCorners2d);
		warpPerspective(imbw, canonicalImg, M, m_markerSize);

		int numNoZero = countNonZero(canonicalImg);
		//cout<<"numNoZero:"<<numNoZero<<endl;
		//imshow("canonicalImg", canonicalImg);
		if(numNoZero>1300)continue;

		//Hue match
		
		matchN=matchShapes(contours[i],matchContour,CV_CONTOURS_MATCH_I1,0);
		/*
		stringstream ss; 
		ss<<matchN;
		setLabel(imcontour,ss.str(),contours[i]);
		*/
		//cout<<"hu: "<<matchN<<endl;
		
		if(matchN<matchMax)
		{
				matchMax=matchN;
				goodContour=i;
				isTracking=true;
				//push keypoint
				goodKeyPt.clear();
				for(int k=0;k<approx.size();k++)
				{
					goodKeyPt.push_back(approx[k]);
				}
		}		
	}
	if(matchN>0.5)isTracking=false;
}

void IrobotDetector::BinBasedDetector()
{
	matchN=10;
	matchMax=100;
	goodContour=-1;

	findContours(imbw.clone(),contours,hierarchy,RETR_TREE, CHAIN_APPROX_SIMPLE);
	for(int i=0;i<contours.size();i++)
	{
			
		//area detect
		if(contourArea(contours[i])<(2000/SCALE/SCALE)||contourArea(contours[i])>(120000/SCALE/SCALE)) continue;
		//drawContours(imcontour,contours,i,Scalar(255,0,0),2);
		//approx ploy
		vector<cv::Point> approx;
		approxPolyDP(contours[i],approx,arcLength(contours[i],true)*0.02,true);
		if(approx.size()!=8) continue;
		//cout<<"8 points pass"<<endl;

		// Convex?
		if (isContourConvex(approx)) continue;
		//cout<<"!isContourConvex pass"<<endl;

		// find the max 
		float maxDist = 0;
		int markerOrigin;
		for (int i=0; i<8; i++)
		{
			Point vecTemp = approx[i] - approx[(i+1)%8];
			float distSquared = vecTemp.dot(vecTemp);
			if(distSquared>maxDist)
			{
				markerOrigin=i;
				maxDist=distSquared;
			}
		}
		//maxDist
		if(maxDist<(3000/SCALE))continue;
			
		//cout<<"maxDist pass"<<endl;
		//right order of point
		rotate(approx.begin(), approx.begin() + markerOrigin, approx.end());
		/*
		for(int j=0;j<approx.size();j++)
		{
			Scalar colorSc(0,255,0);
			circle(imcontour,Point(approx[j].x,approx[j].y),5,colorSc,3);
			stringstream ss2;
			ss2<<j;
			putText(imcontour, ss2.str(), Point(approx[j].x+10,approx[j].y+10), cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255), 2, 8);
		}
		*/
		// cos=90
		//double cos=angle(approx[0], approx[2],approx[1]);
		//cout<<"cos: "<<cos<<endl;
		//if(cos>0.5||cos<-0.7) continue;


		//perspectiveTransform
		Mat canonicalImg;
		vector <Point2f> makerDetect;
		makerDetect.push_back(approx[7]);
		makerDetect.push_back(approx[2]);
		makerDetect.push_back(approx[1]);
		makerDetect.push_back(approx[0]);
		Mat M = getPerspectiveTransform(makerDetect, markCorners2d);
		warpPerspective(imbw, canonicalImg, M, m_markerSize);

		int numNoZero = countNonZero(canonicalImg);
		//cout<<"numNoZero:"<<numNoZero<<endl;
		//imshow("canonicalImg", canonicalImg);
		if(numNoZero>1300)continue;

		//Hue match
		
		matchN=matchShapes(contours[i],matchContour,CV_CONTOURS_MATCH_I1,0);
		/*
		stringstream ss; 
		ss<<matchN;
		setLabel(imcontour,ss.str(),contours[i]);
		*/
		//cout<<"hu: "<<matchN<<endl;
		if(matchN<matchMax)
		{
				matchMax=matchN;
				goodContour=i;
				isTracking=true;
				//push keypoint
				goodKeyPt.clear();
				goodKeyPt.push_back(approx[1]);
				goodKeyPt.push_back(approx[0]);
				goodKeyPt.push_back(approx[7]);
				goodKeyPt.push_back(approx[6]);
				goodKeyPt.push_back(approx[5]);
				goodKeyPt.push_back(approx[4]);
				goodKeyPt.push_back(approx[3]);
				goodKeyPt.push_back(approx[2]);

		}
	}
	if(matchN>0.5)isTracking=false;

}

void IrobotDetector::HarrisBasedDetector()
{
	matchN=10;
	//������1��ûƥ�䵽��Ҳû���Ƶ����ͷ�������
	if(goodKeyPtTime<1)
	{
		goodKeyPtTime++;
		bool flag=false;
		goodFeaturesToTrack( imgray,
				harrisCorners,
				20,
				0.01,
				12,
				Mat(),
				3,
				true,
				0.04 );
		/*
		for(int i=0;i<harrisCorners.size();i++)
		{
			circle(imcontour,Point(harrisCorners[i].x,harrisCorners[i].y),6,Scalar(255,255,255),4);
		}
		*/
		for(int ii=0;ii<harrisCorners.size();ii++)
		{
			if(flag)break;
			for(int jj=0;jj<goodKeyPt.size();jj++)
			{
				int dx=harrisCorners[ii].x-goodKeyPt[jj].x;
				int dy=harrisCorners[ii].y-goodKeyPt[jj].y;
				if(abs(dx)<3&&abs(dy)<3)
				{

					for(int kk=0;kk<goodKeyPt.size();kk++)
					{
						goodKeyPt[kk].x+=dx;
						goodKeyPt[kk].y+=dy;
						isTracking=true;
									
					}

					//Hue match
					
					matchN=matchShapes(goodKeyPt,matchContour,CV_CONTOURS_MATCH_I1,0);
					/*
					stringstream ss; 
					ss<<matchN;
					setLabel(imcontour,ss.str(),goodKeyPt);
					*/
					//cout<<"hu: "<<matchN<<endl;
					flag=true;
					goodKeyPtTime--;
					break;
				}
			}
		}
	}
	if(matchN>0.5)isTracking=false;
}

void IrobotDetector::MatchTemplateBaseDetector()
{
	matchN=10;
	//����matchTemplate
	if(goodKeyPt.size()&&goodKeyPtTime<2)
	{
		goodKeyPtTime++;
		for(int i=0;i<goodKeyPt.size();i++)
		{
			if(goodKeyPt[i].x<11||goodKeyPt[i].y<11||goodKeyPt[i].x>=imgrayTemplate.cols-10||goodKeyPt[i].y>=imgrayTemplate.rows-10){break;}
			Rect rect;
			rect=Rect(goodKeyPt[i].x-10,goodKeyPt[i].y-10,20,20);
			rect &= cv::Rect(0, 0, imgrayTemplate.cols, imgrayTemplate.rows);
			if(tempFlag)
			{
				imTeplate[i]=imgrayTemplate(rect);
				if(i==7) tempFlag=false;
			}
			trackKeyPt(imgrayTemplate,imTeplate[i],rect);
			goodKeyPt[i].x=rect.x+10;
			goodKeyPt[i].y=rect.y+10;
			//rectangle(imcontour, rect, CV_RGB(0,255,0));
			if(i==7)
			{
				isTracking=true;
				goodKeyPtTime--;
				
				//Hue match
				
				matchN=matchShapes(goodKeyPt,matchContour,CV_CONTOURS_MATCH_I1,0);
				/*
				stringstream ss; 
				ss<<matchN;
				setLabel(imcontour,ss.str(),goodKeyPt);
				*/
				//cout<<"hu: "<<matchN<<endl;
				
			}
		}
	}
	if(matchN>0.5)isTracking=false;
}


void IrobotDetector::setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	//cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(255,255,255), thickness, 8);
}

double IrobotDetector::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void IrobotDetector::trackKeyPt(cv::Mat& im, cv::Mat& tpl, cv::Rect& rect)
{
	//cv::Size size(rect.width * 2, rect.height * 2);
	//cv::Rect window(rect + size - cv::Point(size.width/2, size.height/2));
	Rect window;
	window.x=rect.x-6;
	window.y=rect.y-6;
	window.width=window.height=rect.width+12;
	window &= cv::Rect(0, 0, im.cols, im.rows);
	int xx=window.width - tpl.rows + 1;
	int yy=window.height - tpl.cols + 1;
	if(xx<=0) xx=1;
	if(yy<=0) yy=1;
	cv::Mat dst(xx,yy, CV_32FC1);

	//
	//cv::matchTemplate(im(window), tpl, dst, CV_TM_SQDIFF_NORMED);

	//ncc
	cv::matchTemplate(im(window), tpl, dst, CV_TM_CCOEFF_NORMED);

	//imshow("im",im(window));
	////imshow("tpl",tpl);
	////imshow("dst",dst);
	//
	double minval, maxval;
	cv::Point minloc, maxloc;
	cv::minMaxLoc(dst, &minval, &maxval, &minloc, &maxloc);

	//sqd
	//rect.x = window.x + minloc.x;
	//rect.y = window.y + minloc.y;

	//ncc
	rect.x = window.x + maxloc.x;
	rect.y = window.y + maxloc.y;
}


void IrobotDetector::estimatePosition(vector<cv::Point3f> Pt3d,vector<cv::Point2f> Pt2d)
{
		//Mat_<float> Tvec;
		
		// ����һ��3d to 2d��ӳ��
		if(guess)
		{
			solvePnP(Pt3d, Pt2d, camMatrix, distCoeff, raux, taux,true);
			//solvePnPRansac(Pt3d, Pt2d, camMatrix, distCoeff, raux, taux,true);
			//std::cout<<"Guess!!!!!!!!!!!!"<<std::endl;
		}
		else
		{
			//solvePnPRansac(Pt3d, Pt2d, camMatrix, distCoeff, raux, taux);
			solvePnP(Pt3d, Pt2d, camMatrix, distCoeff, raux, taux);
			//guess=true;
		}


		
		taux.convertTo(Tvec, CV_32F);
		raux.convertTo(Rvec, CV_32F);
#if(DEBUG_ON)

		
// 		Mat_<float> rotMat(3,3);
		Matx33f rotMat;
		Rodrigues(Rvec, rotMat);

		drawR = rotMat.t();
		//std::cout<<"��ת����: "<<drawR<<std::endl;
		//drawR = rotMat.inv();
		//std::cout<<"��ת����: "<<drawR<<std::endl;

		drawT = -Tvec;

		//system("pause");
		Vec3f Tvec2;
		Tvec2=-drawR*Tvec;  

		/*
		Vec3f xxx,xxx2;
		Matx33f yyy;

		xxx<<1,1,1;
		yyy<<1,0,0,
			 0,2,0,
			 0,0,3;
		xxx2=yyy*xxx;
		cout<<xxx2;
		*/
		//cout<<"T_ct: "<<Tvec[0]<<","<<Tvec[1]<<","<<Tvec[2]<<endl;

		//std::cout<<"T_ct: "<<Tvec<<std::endl;
		//std::cout<<"T_tc: "<<Tvec2<<std::endl;

		//std::cout<<"-Tvec: "<<drawT<<std::endl;
		//std::cout<<"��ת����: "<<drawR<<std::endl;
		//std::cout<<"���룺    "<<sqrt(Tvec[0]*Tvec[0]+Tvec[1]*Tvec[1]+Tvec[2]*Tvec[2])<<std::endl;
#endif

}


void IrobotDetector::drawPlain(Mat& _frame)
{
	vector<Point3f> cube3D;
	vector<Point2f> cube2D;
	cube2D.clear();
	cube3D.clear();
	cube3D.push_back(Point3f(-0.5f,-0.5f,0));
	cube3D.push_back(Point3f(+0.5f,-0.5f,0));
	cube3D.push_back(Point3f(+0.5f,+0.5f,0));
	cube3D.push_back(Point3f(-0.5f,+0.5f,0));
	cube3D.push_back(Point3f(-0.5f,-0.5f,0.25));
	cube3D.push_back(Point3f(+0.5f,-0.5f,0.25));
	cube3D.push_back(Point3f(+0.5f,+0.5f,0.25));
	cube3D.push_back(Point3f(-0.5f,+0.5f,0.25));
	cube3D.push_back(Point3f(0.f,0.f,0.f));
	
	Mat Tvec=(Mat_<float>(3,1)<<0,0,-14);
	Mat Rvec;
	raux.convertTo(Rvec, CV_32F);
	projectPoints(cube3D,Rvec,Tvec,camMatrix,distCoeff,cube2D);

	line(_frame,cube2D[0],cube2D[1],Scalar(255,255,0),2);
	line(_frame,cube2D[1],cube2D[2],Scalar(255,255,0),2);
	line(_frame,cube2D[2],cube2D[3],Scalar(255,255,0),2);
	line(_frame,cube2D[3],cube2D[0],Scalar(255,255,0),2);
	line(_frame,cube2D[0],cube2D[2],Scalar(255,255,0),2);
	line(_frame,cube2D[1],cube2D[3],Scalar(255,255,0),2);

	line(_frame,cube2D[0],cube2D[4],Scalar(255,255,0),2);
	line(_frame,cube2D[1],cube2D[5],Scalar(255,255,0),2);
	line(_frame,cube2D[2],cube2D[6],Scalar(255,255,0),2);
	line(_frame,cube2D[3],cube2D[7],Scalar(255,255,0),2);

	circle(_frame,cube2D[4],3,Scalar(0,255,255  ),4,8);
	circle(_frame,cube2D[5],3,Scalar(255,0,255  ),4,8);
	circle(_frame,cube2D[6],3,Scalar(255,100,100),4,8);
	circle(_frame,cube2D[7],3,Scalar(100,100,255),4,8);

}


void IrobotDetector::drawGoodpt(Mat image,vector<cv::Point> goodpt,Scalar a)
{
	for(int j=0;j<goodpt.size();j++)
	{
		circle(image,Point(goodpt[j].x,goodpt[j].y),1,a,2);
		stringstream ss2;
		ss2<<j;
		putText(image, ss2.str(), Point(goodpt[j].x+10,goodpt[j].y+10), cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255), 1, 8);
	}

}
