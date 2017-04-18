#include "Markdetection.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <stdio.h>
#include<string.h>
#include<malloc.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include"math.h"

#include <errno.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
using namespace cv;

using namespace std;
const char* wndname = "Square Detection Demo";
volatile double ry,rz,rx;
volatile double tx,ty,tz;

int grey_thresh=100;
int numframe=0;   //frame numbers
int lost_numframe=0;
double omission=0.0;
#ifdef Camera_One
//for 800*600
//camera 1
Mat cameraMatrix=(Mat_<double>(3,3)<<942.6637,0,342.8562,0,943.0159,320.2033,0,0,1);

Mat distCoeffs=(Mat_<double>(1,4)<<-0.2248,11.8088,-0.0071,0.0045);

#else

//camera 2
Mat cameraMatrix=(Mat_<double>(3,3)<<1079.2096,0,342.5224,0,1075.3261,353.0309,0,0,1);

Mat distCoeffs=(Mat_<double>(1,4)<<-0.4513,0.1492,-0.0030,0.0043);
#endif

const int ARROW_AREA_MIN=2000;//variable
const int ARROW_AREA_MAX=20000;
Mat pro_after;
int cx=343;   //To change to calibration parameter.
int cy=320;   //the same with cameraMatrix.cx,cy

Point3f world_pnt_tl(-65,-85,0);   //unit: mm
Point3f world_pnt_tr(65,-85,0);
Point3f world_pnt_br(65,85,0);
Point3f world_pnt_bl(-65,85,0);

double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void findSquares( Mat src,const Mat& image, vector<vector<Point> >& squares )
{
    Mat pyr, timg, gray0(image.size(), CV_8U),Src_HSV(image.size(), CV_8U), gray;

    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    int counts=0,counts_2=0,counts_3=0;
    vector<Point> cand_1,cand_2,cand_3;
    vector<vector<Point>> rect_2;
    vector<vector<Point>> rect_3;
    vector<vector<Point>> out;
    
	//cout<<"find squares src.chan="<<src.channels()<<endl;
	cvtColor( src, gray0, CV_BGR2GRAY);
    
            Canny(gray0, gray, 50, 200, 5);
	    
#ifdef _SHOW_PHOTO
	    imshow("Canny", gray);
#endif

	    //dilate(gray, gray, Mat(), Point(-1,-1));
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
            
	    vector<Point> approx;
            
	    for( size_t i = 0; i < contours.size(); i++ )
            {
                if (contours[i].size() < 50 || contours[i].size() > 400)
                {
                    continue;
                }
                //cout<<"rectangle detected 1111"<<endl;
                
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.03, true);
		
		//cout<<approx.size()<<" "<< fabs(contourArea(Mat(approx))) << " " <<  isContourConvex(Mat(approx)) <<endl;
                
		if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)) )
                {
		    //cout<<"rectangle detected  2222"<<endl;
                    double maxCosine = 0;
                    for( int j = 2; j < 5; j++ )
                    {
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    if( maxCosine < 0.3 )
                    {
                        counts++;
                        int tl_x,tl_y,br_x,br_y;
			int tl_x_2,tl_y_2,br_x_2,br_y_2;
			int tl_x_3,tl_y_3,br_x_3,br_y_3;
                        Sort_rect(approx);
                        if(counts>100)
                            counts=counts%10+2;
                        if(counts==1)
                        {
                            cand_1=approx;
                            tl_x=approx[0].x;
                            tl_y=approx[0].y;
                            br_x=approx[2].x;
                            br_y=approx[2].y;
#ifdef _SHOW_OUTPUT
                            cout<<"rect_1 come ! "<<endl;
                            //cout<<"static int tl x: "<<tl_x<<endl;
                            //cout<<"static int tl y: "<<tl_y<<endl;
                            //cout<<"static int br x: "<<br_x<<endl;
                            //cout<<"static int br y: "<<br_y<<endl;
#endif
                        }
                        else
                        { 
                            if(abs(tl_x-approx[0].x)<10&&abs(tl_y-approx[0].y)<10&&abs(br_x-approx[2].x)<10&&abs(br_y-approx[2].y)<10)
                            continue;
                        else
                        {
                            counts_2++;
                            if(counts_2>100)
                                counts_2=counts_2%10+2;
                            if(counts_2==1)
                            {
                                cand_2=approx;
                                tl_x_2=approx[0].x;
                                tl_y_2=approx[0].y;
                                br_x_2=approx[2].x;
                                br_y_2=approx[2].y;
#ifdef _SHOW_OUTPUT
                                cout<<"rect_2 come ! "<<endl;
                                //cout<<"static int tl x: "<<tl_x_2<<endl;
                                //cout<<"static int tl y: "<<tl_y_2<<endl;
                                //cout<<"static int br x: "<<br_x_2<<endl;
                                //cout<<"static int br y: "<<br_y_2<<endl;
#endif
                            }
                            else
                            {
                                if(abs(tl_x_2-approx[0].x)<10&&abs(tl_y_2-approx[0].y)<10&&abs(br_x_2-approx[2].x)<10&&abs(br_y_2-approx[2].y)<10)
                                    continue;
                                else
                                {
                                    counts_3++;
                                    if(counts_3>100)
                                        counts_3=counts_3%10+2;
                                    if(counts_3==1)
                                    {
                                        cand_3=approx;
                                        tl_x_3=approx[0].x;
                                        tl_y_3=approx[0].y;
                                        br_x_3=approx[2].x;
                                        br_y_3=approx[2].y;
#ifdef _SHOW_OUTPUT
                                        cout<<"rect_3 come ! "<<endl;
                                        //cout<<"static int tl x: "<<tl_x_3<<endl;
                                        //cout<<"static int tl y: "<<tl_y_3<<endl;
                                        //cout<<"static int br x: "<<br_x_3<<endl;
                                        //cout<<"static int br y: "<<br_y_3<<endl;
#endif
                                        if(abs(tl_x_3-approx[0].x)<10&&abs(tl_y_3-approx[0].y)<10&&abs(br_x_3-approx[2].x)<10&&abs(br_y_3-approx[2].y)<10)
                                            continue;
                                        else
                                            out.push_back(approx);
                                    }
                                }
                            }
                            }
                        }
                    }
                }
            }
    if(cand_1.size()>0)
    {
        
        Rect roi(cand_1[0],cand_1[2]);
		
       Mat ROI_image(src,roi); //src(roi).copyTo(ROI_image);
       if(ROI_image.rows != 0 && ROI_image.cols != 0)
	   {
	   
		   int rect_1_true=Color_judge(ROI_image,(cand_1[2].x-cand_1[0].x)*(cand_1[2].y-cand_1[0].y));
			//cout<<"rect1 true: "<<rect_1_true<<endl;
        if(rect_1_true)
            squares.push_back(cand_1);
		}
	   
        
        if(cand_2.size()>0)
        {
            //Mat ROI_image_2;
            Rect roi_2(cand_2[0],cand_2[2]);
			 Mat ROI_image_2(src,roi_2);
            //src(roi_2).copyTo(ROI_image_2);
			 if(ROI_image_2.rows != 0 && ROI_image_2.cols != 0)
			 {
				 int rect_2_true=Color_judge(ROI_image_2,(cand_2[2].x-cand_2[0].x)*(cand_2[2].y-cand_2[0].y));
				//cout<<"rect2 true: "<<rect_2_true<<endl;
            if(rect_2_true)
                squares.push_back(cand_2);
			 }
            
            if(cand_3.size()>0)
            {
                
                Rect roi_3(cand_3[0],cand_3[2]);
				Mat ROI_image_3(src,roi_3);
               // src(roi_3).copyTo(ROI_image_3);
				if(ROI_image_3.rows != 0 && ROI_image_3.cols != 0)
				{
					int rect_3_true=Color_judge(ROI_image_3,(cand_3[2].x-cand_3[0].x)*(cand_3[2].y-cand_3[0].y));
					//cout<<"rect3 true: "<<rect_3_true<<endl;
                if(rect_3_true)
                    squares.push_back(cand_3);
				}
                
                if(out.size()>0)
                    cout<<"too much rect ! "<<endl;
            }
        }
    }
}
void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
#ifdef _SHOW_PHOTO
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);

#endif
    }
    //cout<<"squares.size: "<<squares.size()<<endl;
	
	//caculate the omission rate
	numframe++;


	if(squares.size()==0)
		lost_numframe++;
	if(numframe>=50)
	{
		omission=((double)lost_numframe)/(double)numframe;
		numframe=0;
		lost_numframe=0;
	}
#ifdef _SHOW_PHOTO
        char str_y[20];
        char str_z[20];
        char str_x[20];
        char str_tz[20];
        char str_ty[20];
        char str_tx[20];
		char str_om[20];
        sprintf(str_y,"%lf",ry);
        sprintf(str_z,"%lf",rz);
        sprintf(str_x,"%lf",rx);
        sprintf(str_tz,"%lf",tz);
        sprintf(str_ty,"%lf",ty);
        sprintf(str_tx,"%lf",tx);
		sprintf(str_om,"%lf",omission);
        string pre_str_y="thetay: ";
        string pre_str_z="thetaz: ";
        string pre_str_x="thetax: ";
        string pre_str_tz="tz: ";
        string pre_str_ty="ty: ";
        string pre_str_tx="tx: ";
		string pre_str_om="omiss: ";
        string full_y=pre_str_y+str_y;
        string full_z=pre_str_z+str_z;
        string full_x=pre_str_x+str_x;
        string full_tz=pre_str_tz+str_tz;
        string full_ty=pre_str_ty+str_ty;
        string full_tx=pre_str_tx+str_tx;
		string full_om=pre_str_om+str_om;
        putText(image,full_y,Point(30,30),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
        putText(image,full_z,Point(30,70),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
        putText(image,full_x,Point(30,100),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
        putText(image,full_tz,Point(30,130),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
        putText(image,full_ty,Point(30,170),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
        putText(image,full_tx,Point(30,200),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
		putText(image,full_om,Point(30,240),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
#endif
 
#ifdef _SHOW_PHOTO
    imshow("Square Detection Demo", image);
#endif
}

void LocationMarkes(const vector<vector<Point> >& squares)
{
	if(squares.size()>0)
    {
        Point tl,tr,br,bl;
        int max=squares[0][0].y;
        int id=0;
        for(int i=1; i<squares.size(); i++)
        {
            if(squares[i][0].y>max)
            {
                max=squares[i][0].y;
                id=i;
            }
        }
        tl=squares[id][0];
        tr=squares[id][1];
        br=squares[id][2];
        bl=squares[id][3];
        Calcu_attitude(world_pnt_tl,world_pnt_tr,world_pnt_br,world_pnt_bl,tl,tr,br,bl);
    }
}



void Calcu_attitude(Point3f world_pnt_tl,Point3f world_pnt_tr,Point3f world_pnt_br,Point3f world_pnt_bl,Point2f pnt_tl_src,Point2f pnt_tr_src,Point2f pnt_br_src,Point2f pnt_bl_src)
{
    vector<Point3f> Points3D;
    vector<Point2f> Points2D;
    // Vec3d  rvec,tvec;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    Points3D.push_back(world_pnt_tl);
    Points3D.push_back(world_pnt_tr);
    Points3D.push_back(world_pnt_br);
    Points3D.push_back(world_pnt_bl);
    Points2D.push_back(pnt_tl_src);
    Points2D.push_back(pnt_tr_src);
    Points2D.push_back(pnt_br_src);
    Points2D.push_back(pnt_bl_src);
    solvePnP(Points3D,Points2D,cameraMatrix,distCoeffs,rvec,tvec);
    //Mat rotM(3,3,CV_64FC1);
    double rm[9];
    cv::Mat rotM(3, 3, CV_64FC1, rm);
    Rodrigues(rvec,rotM);

    double r11 = rotM.ptr<double>(0)[0];
    double r12 = rotM.ptr<double>(0)[1];
    double r13 = rotM.ptr<double>(0)[2];
    double r21 = rotM.ptr<double>(1)[0];
    double r22 = rotM.ptr<double>(1)[1];
    double r23 = rotM.ptr<double>(1)[2];
    double r31 = rotM.ptr<double>(2)[0];
    double r32 = rotM.ptr<double>(2)[1];
    double r33 = rotM.ptr<double>(2)[2];

    double thetaz = atan2(r21, r11) / CV_PI * 180;
    double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
    double thetax = atan2(r32, r33) / CV_PI * 180;
    ry=thetay;
    rz=thetaz;
    rx=thetax;
    tx=tvec.ptr<double>(0)[0];
    ty=tvec.ptr<double>(1)[0];
    tz=tvec.ptr<double>(2)[0];
#ifdef _SHOW_OUTPUT
    cout<<"tvec: "<<tvec<<endl;
    cout<<"thetay: "<<thetay<<endl;
    cout<<"thetaz: "<<thetaz<<endl;
    cout<<"thetax: "<<thetax<<endl;
#endif
    
}
int Color_judge(Mat &src,int area)
{
  //judge from the center point: BGR
  Mat dst;
  if(src.rows == 0 && src.cols == 0)
  {
	  return 0;
  }
  //cout<<"color judges 1 src.chan="<<src.channels()<<endl;
  cvtColor( src, dst, CV_BGR2HSV);
  //src = dst;
  
  int x=src.cols/2;
  int y=src.rows/2;
  //cout<<"RGB of Center ("<<y<<","<<x<<")="<<(int)dst.at<Vec3b>(y,x)[0]<<" "<<(int)dst.at<Vec3b>(y,x)[1]<<" "<<(int)dst.at<Vec3b>(y,x)[2]<<endl;
  if((int)dst.at<Vec3b>(y,x)[0]>80&&(int)dst.at<Vec3b>(y,x)[0]<120) //src.at<Vec3b>(y,x)[2]<50&&src.at<Vec3b>(y,x)[1]>50&&src.at<Vec3b>(y,x)[1]<200&&src.at<Vec3b>(y,x)[0]>100)
  {
    Mat grey;
    cvtColor(src,grey,CV_BGR2GRAY);
    const int channels[1]= {0};
    const int histSize[1]= {256};
    float hranges[2]= {0,255};
    const float* ranges[1]= {hranges};
    MatND hist;
    calcHist(&grey,1,channels,Mat(),hist,1,histSize,ranges);
    //cout<<"hist: "<<hist<<endl;
    float sum=0;
    for(int i=0; i<grey_thresh; i++)
    {
      sum+=hist.at<float>(i);
    }
    //cout<<"sum: "<<sum<<endl;
    //cout<<"portion: "<<sum/area<<endl;
    if(sum/area>0.7&&sum/area<0.95) 
      return 1;
    else return 0;
  }
  else 
    return 0;

}
void Sort_rect(vector<Point>& approx)
{
    Point tl,tr,br,bl;
    vector<MyPoint> my_rect;
    my_rect.push_back(approx[0]);
    my_rect.push_back(approx[1]);
    my_rect.push_back(approx[2]);
    my_rect.push_back(approx[3]);
    sort(my_rect.begin(),my_rect.end());
    if(my_rect[0].y<my_rect[1].y)
    {
        tl.x=my_rect[0].x;
        tl.y=my_rect[0].y;
        bl.x=my_rect[1].x;
        bl.y=my_rect[1].y;
    }
    else
    {
        tl.x=my_rect[1].x;
        tl.y=my_rect[1].y;
        bl.x=my_rect[0].x;
        bl.y=my_rect[0].y;
    }
    if(my_rect[2].y<my_rect[3].y)
    {
        tr.x=my_rect[2].x;
        tr.y=my_rect[2].y;
        br.x=my_rect[3].x;
        br.y=my_rect[3].y;
    }
    else
    {
        tr.x=my_rect[3].x;
        tr.y=my_rect[3].y;
        br.x=my_rect[2].x;
        br.y=my_rect[2].y;
    }
    approx[0]=tl;
    approx[1]=tr;
    approx[2]=br;
    approx[3]=bl;
#ifdef _SHOW_OUTPUT
    cout<<"sorted approx: "<<approx<<endl;
#endif
}
/************************************************************/
/**********************detect the blue area******************/
/**********************Input:image with blue arrow inside *****/
/*********************Output: the center(x,y) ofthe blue area*********/
int Color_detect(Mat frame, int &diff_x, int &diff_y)
{
    vector<Mat> HSVSplit;
    //Returns a rectangular structuring element of the specified size and shape for morphological operations.
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    vector<vector<Point> > contours;
    Mat HSVImage;
    Mat out;
    Mat HGreen;
    Mat SGreen;
    Rect r;
    Rect max_tmp;
    resize(frame,frame,Size(640,480));
    //imshow("src",frame);
    cvtColor(frame, HSVImage, CV_BGR2HSV);
    split(HSVImage,HSVSplit);
    //Hgreen=HSVSplit[0]>lower&&HSVSplit[0]<up , mask, threshold can be fine tuned.
    inRange(HSVSplit[0], Scalar(80), Scalar(120), HGreen);
	//inRange(HSVSplit[1], Scalar(80), Scalar(255), SGreen);
    threshold(HSVSplit[1], SGreen, 80, 255, THRESH_BINARY);    //S channal intensity
    //bitwise conjunction
    cv::bitwise_and(HGreen, SGreen, out);
    morphologyEx(out, out, MORPH_OPEN, element);//open operator,remove isolated noise points.
    morphologyEx(out, out, MORPH_CLOSE, element);//close operator,get connected domain.BMC
    Mat solid;
    solid=out.clone();
    findContours(out,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    if( contours.size() == 0 )
    {
        diff_x=DIF_CEN;
   	diff_y=0;
        cout<<"no contour..."<<endl;
        return 0;
    }
    Mat result(out.size(),CV_8U,Scalar(0));
    drawContours(result,contours,-1,Scalar(255),2);
    max_tmp=boundingRect(Mat(contours[0]));
    for(int i=1; i<contours.size(); i++)    
    {
        r = boundingRect(Mat(contours[i]));
        max_tmp=(r.area()>max_tmp.area())?r:max_tmp;
    }

    cout<<"area "<<max_tmp.area()<<endl;
    if(max_tmp.area()<ARROW_AREA_MIN||max_tmp.area()>ARROW_AREA_MAX)
	{
		diff_x=DIF_CEN;
                cout<<"area...."<<endl;
		diff_y=0;
        return 0;
	}
    Mat pro;
    solid(max_tmp).copyTo(pro);
    pro_after=pro.clone();
    rectangle(result, max_tmp, Scalar(255), 2);
    
//#ifdef _SHOW_PHOTO
    imshow("result",result);
//#endif
   // if('q'==(char)waitKey(7)) exit(0);
    
    //caculate the center of green area
    Moments mt;
    mt=moments(pro_after,true);
    Point center;
	
    center.x=mt.m10/mt.m00+max_tmp.tl().x;
    center.y=mt.m01/mt.m00+max_tmp.tl().y;
    cout<<"width="<<mt.m10/mt.m00<<"  height="<<mt.m01/mt.m00<<endl;
    diff_x=center.x*800/640-cx;
    diff_y=center.y*600/480-cy;
    cout<<"diff x: "<<diff_x<<endl;
    cout<<"diff y: "<<diff_y<<endl;
    return 1;
}
