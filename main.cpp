#include "main.h"
#include "RMVideoCapture.hpp"

#define COM_PORT 1 //"COM7"
#define BUF_LEN 256
#define TIMEOUT 30
#define FRAME_N 20000


#define BUFFER_EXPOSURE_VALUE 150
//#define DEFAULT_FRAME_WIDTH 1280
//#define DEFAULT_FRAME_HEIGHT 720
#define DEFAULT_FRAME_WIDTH 640
#define DEFAULT_FRAME_HEIGHT 480
#define _SHOW_PHOTO
//#define _SHOW_OUTPUT
//#define Camera_One

using namespace cv;

using namespace std;



int thresh = 50, N =1;
const char* wndname = "Square Detection Demo";

 int grey_thresh=100;
//int grey_thresh=50;

double ry,rz,rx;
double tx,ty,tz;
/*
//for 640*480 
Mat cameraMatrix=(Mat_<double>(3,3)<<569.2681,0,288.1437,0,569.4591,263.6782,0,0,1);
//for 640x480.
Mat distCoeffs=(Mat_<double>(1,4)<<0.0344,-0.0085,-0.0032,-0.0028);
*/
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

Point3f world_pnt_tl(-65,-85,0);   //unit: mm
Point3f world_pnt_tr(65,-85,0);
Point3f world_pnt_br(65,85,0);
Point3f world_pnt_bl(-65,85,0);

void Calcu_attitude(Point3f world_pnt_tl,Point3f world_pnt_tr,Point3f world_pnt_br,Point3f world_pnt_bl,Point2f pnt_tl_src,Point2f pnt_tr_src,Point2f pnt_br_src,Point2f pnt_bl_src);
int Color_judge(Mat &src,int area);
void Sort_rect(vector<Point>& approx);
class MyPoint
{
public:
    MyPoint(Point pnt)
    {
        x=pnt.x;
        y=pnt.y;
    };
    int x;
    int y;
    bool operator<(const MyPoint&p)const
    {
        return x<p.x;
    }
};




void VRC_Proc(const VirtualRC_t* vrc)
{
	printf("*************************************VRC********************************************\n");
	printf("ch0=%d,ch1=%d,ch2=%d,ch3=%d,s1=%d,s2=%d\n",
		vrc->ch[0], vrc->ch[1], vrc->ch[2], vrc->ch[3], vrc->sw[0], vrc->sw[1]);
	//printf("*************************************VRC********************************************\n");
}

void VHC_Proc(const VirtualHC_t* vhc)
{
	printf("*************************************VHC********************************************\n");
	printf("k=%d,x=%d,y=%d,z=%d,l=%d,r=%d\n",
		vhc->key.val, vhc->mouse.x, vhc->mouse.y, vhc->mouse.z,
		vhc->mouse.b[0], vhc->mouse.b[1]);
	//printf("*************************************VHC********************************************\n");
}

void VDBUS_Proc(const VirtualDBUS_t* vdbus)
{
	//printf("*************************************VDBUS********************************************\n");
	printf("ch0=%d,ch1=%d,ch2=%d,ch3=%d,s1=%d,s2=%d,k=%d,x=%d,y=%d,z=%d,l=%d,r=%d\n",
		vdbus->rcp.ch[0], vdbus->rcp.ch[1], vdbus->rcp.ch[2], vdbus->rcp.ch[3], vdbus->rcp.sw[0],
		vdbus->rcp.sw[1], vdbus->hcp.key.val, vdbus->hcp.mouse.x, vdbus->hcp.mouse.y, vdbus->hcp.mouse.z,
		vdbus->hcp.mouse.b[0], vdbus->hcp.mouse.b[1]);
	//printf("*************************************VDBUS********************************************\n");
}

void VCBUS_Proc(const VirtualCBUS_t* vcbus)
{
}

static void Dnl_ProcCalibMsg(const CalibMsg_t* calibMsg)
{
	//printf("*************************************CALIB********************************************\n");
	//printf("calib_msg=%x\n", calibMsg->auto_cali_flag);
	//printf("kp=%d,ki=%d,kd=%d,eh=%d,el=%d,ph=%d,pl=%d\n", calibMsg->pid.kp, calibMsg->pid.ki, calibMsg->pid.kd, calibMsg->pos.elevator_h, calibMsg->pos.elevator_l, calibMsg->pos.pwm_h, calibMsg->pos.pwm_l);
	//printf("*************************************CALIB********************************************\n");
}

static void Dnl_ProcMotorMsg(const MotorMsg_t* motorMsg)
{
	//printf("*************************************MOTOR********************************************\n");
	printf("id=%d,ecd_angle=%d,angle=%d,rate=%d,current=%d\n", motorMsg->id, motorMsg->ecd_angle, motorMsg->angle, motorMsg->rate, motorMsg->current);
}

static void Dnl_ProcOdomeMsg(const OdomeMsg_t* odomeMsg)
{
	//printf("*************************************ODOME********************************************\n");
	printf("px=%d,py=%d,pz=%d,vx=%d,vy=%d,vz=%d\n", odomeMsg->px, odomeMsg->py, odomeMsg->pz, odomeMsg->vx, odomeMsg->vy, odomeMsg->vz);
}

static void Dnl_ProcStatuMsg(const StatuMsg_t* statuMsg)
{
	//printf("*************************************STATU********************************************\n");
	printf("wgd=%d,ini=%d\n", statuMsg->wdg, statuMsg->ini);
}

static void Dnl_ProcZGyroMsg(const ZGyroMsg_t* zgyroMsg)
{
	//printf("*************************************ZGYRO********************************************\n");
	printf("angle=%d,rate=%d\n", zgyroMsg->angle, zgyroMsg->rate);
}

static void Dnl_ProcKylinMsg(const KylinMsg_t* kylinMsg)
{
        //printf("*************************************KYLIN********************************************\n");
        printf("fs=%x,px=%d,py=%d,pz=%d,pe=%d,pc=%d,vx=%d,vy=%d,vz=%d,ve=%d,vc=%d\n", kylinMsg->fs,
                kylinMsg->cp.x, kylinMsg->cp.y, kylinMsg->cp.z, kylinMsg->gp.e, kylinMsg->gp.c,
                kylinMsg->cv.x, kylinMsg->cv.y, kylinMsg->cv.z, kylinMsg->gv.e, kylinMsg->gv.c);
}

uint8_t exit_flag = 0;

FIFO_t rx_fifo;
uint8_t rx_buf[2][BUF_LEN];

VirtualDBUS_t vdbus;
VirtualCBUS_t vcbus;
CalibMsg_t calibMsg;
MotorMsg_t motorMsg;
ZGyroMsg_t zgyroMsg;
OdomeMsg_t odomeMsg;
GraspMsg_t graspMsg;
KylinMsg_t kylinMsg;

void PullMsg()
{
	// Get fifo free space
	int len = FIFO_GetFree(&rx_fifo);
	// If fifo free space insufficient, pop one element out
	if (!len) {
		uint8_t b;
		len = FIFO_Pop(&rx_fifo, &b, 1);
	}
	// Read input stream according to the fifo free space left
	len = read_serial(rx_buf[1], len, TIMEOUT);
	//printf("PULL LEN= %d\n", len);
	// Push stream into fifo
	FIFO_Push(&rx_fifo, rx_buf[1], len);
	// Check if any message received
	if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vrc, &vdbus.rcp)) {
		//VRC_Proc(&vdbus.rcp);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vhc, &vdbus.hcp)) {
		//VHC_Proc(&vdbus.hcp);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vdbus, &vdbus)) {
		//VDBUS_Proc(&vdbus);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vcbus, &vcbus)) {
		//VCBUS_Proc(&vcbus);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_calib, &calibMsg)) {
		//Dnl_ProcCalibMsg(&calibMsg);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_motor, &motorMsg)) {
		//Dnl_ProcMotorMsg(&motorMsg);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_zgyro, &zgyroMsg)) {
		//Dnl_ProcZGyroMsg(&zgyroMsg);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_odome, &odomeMsg)) {
		//Dnl_ProcOdomeMsg(&odomeMsg);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_grasp, &graspMsg)) {
		//Dnl_ProcGraspMsg(&graspMsg);
	}
	else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_kylin, &kylinMsg)) {
		//printf("Before pop -> size: %d, used: %d, free: %d\n", FIFO_GetSize(&rx_fifo), FIFO_GetUsed(&rx_fifo), FIFO_GetFree(&rx_fifo));
		//printf("Before pop: %d\n", FIFO_GetFree(&rx_fifo));
		Dnl_ProcKylinMsg(&kylinMsg);
		//printf("After pop: %d\n", FIFO_GetFree(&rx_fifo));
	}
	else {
		//uint8_t b;
		//len = FIFO_Pop(&rx_fifo, &b, 1);
	}
}

void *KylinBotMsgPullerThreadFunc(void* param)
{
    while (exit_flag == 0) {
	      //PullMsg();
	      //printf("H_");
	      usleep(4000);
	}
	return NULL;
}

FIFO_t tx_fifo;
uint8_t tx_buf[2][BUF_LEN];
KylinMsg_t txKylinMsg;
void PushMsg()
{
	//txKylinMsg.fs = 1;
	//txKylinMsg.cv.x = 2000;
	uint32_t len = Msg_Push(&tx_fifo, tx_buf[1], & msg_head_kylin, &txKylinMsg);
	FIFO_Pop(&tx_fifo, tx_buf[1], len);
	write_serial(tx_buf[1], len, TIMEOUT);
}

void *KylinBotMsgPusherThreadFunc(void* param)
{
  while (exit_flag == 0) {
		PushMsg();
		usleep(4000);
	}
}

void init()
{
	FIFO_Init(&rx_fifo, rx_buf[0], BUF_LEN);
	FIFO_Init(&tx_fifo, tx_buf[0], BUF_LEN);
}

typedef struct
{
  uint8_t dir;
  uint32_t cnt;
  uint32_t scl;
}Tri_t;

void Tri_Init(Tri_t* tri, uint32_t scl)
{
  tri->scl = scl;
  tri->cnt = 0;
  tri->dir = 0;
}

float Tri_Proc(Tri_t* tri)
{
  if (tri->dir == 0) {
     if (tri->cnt < tri->scl) {
       tri->cnt++;
     }
     else {
       tri->dir = 1;
     }
  }
  else if (tri->dir == 1) {
     if (tri->cnt > 0) {
       tri->cnt--;
     }
  }
  return (float)tri->cnt / (float)tri->scl;
}

void Tri_Reset(Tri_t* tri)
{
  tri->cnt = 0;
  tri->dir = 0;
  //memset(tri, 0, sizeof(Tri_t));
}

#define MAF_BUF_LEN 20
static uint32_t cnt = 0;
static Rmp_t rmp;
static Tri_t tri;
static Maf_t maf;
static float maf_buf[MAF_BUF_LEN];
static uint8_t dir = 0;
void kylinbot_control()
{
  ++cnt;
  
  if (cnt < 5e4) {
    if (dir != 1) {
      Rmp_Reset(&rmp);
      Tri_Reset(&tri);
      dir = 1;
    }
    // Move forward
    printf("Move forward\n");
    memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
    txKylinMsg.cp.y = 1000;
    txKylinMsg.cv.y = 1000 * Tri_Proc(&tri); //* Rmp_Calc(&rmp);
  }
  else if (cnt < 10e4) {
    if (dir != 2) {
      Rmp_Reset(&rmp);
      Tri_Reset(&tri);
      dir = 2;
    }
    // Move right
    printf("Move right\n");
    memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
    txKylinMsg.cp.x = 1000;
    txKylinMsg.cv.x = 1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
  }
  else if (cnt < 15e4) {
    if (dir != 3) {
      Rmp_Reset(&rmp);
      Tri_Reset(&tri);
      dir = 3;
    }
    // Move backward
    printf("Move backward\n");
    memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
    txKylinMsg.cp.y = -1000;
    txKylinMsg.cv.y = -1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
  }
  else if (cnt < 20e4) {
    if (dir != 4) {
      Rmp_Reset(&rmp);
      Tri_Reset(&tri);
      dir = 4;
    }
    // Move left
    printf("Move left\n");
    memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
    txKylinMsg.cp.x = -1000;
    txKylinMsg.cv.x = -1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
  }
  else {
    cnt = 0;
  }
  
  printf("cnt=%d\n", tri.cnt);
}


void set_camera_exposure(int val) {

    int cam_fd;
    if ((cam_fd = open("/dev/video0", O_RDWR)) == -1) {
        cerr << "Camera open error" << endl;
        exit(0);
    }

    struct v4l2_control control_s;
    control_s.id = V4L2_CID_EXPOSURE_AUTO;
    control_s.value = V4L2_EXPOSURE_MANUAL;
    ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

    control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control_s.value = val;
    ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);
    close(cam_fd);
}

static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static void findSquares( Mat src,const Mat& image, vector<vector<Point> >& squares )
{
    squares.clear();

    Mat pyr, timg, gray0(image.size(), CV_8U),Src_HSV(image.size(), CV_8U), gray;

    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;

    int counts=0,counts_2=0,counts_3=0;
    vector<Point> cand_1,cand_2,cand_3;
    vector<vector<Point>> rect_2;
    vector<vector<Point>> rect_3;
    vector<vector<Point>> out;
    //vector<Mat> channels;
    /*
	cvtColor( timg, Src_HSV, CV_BGR2HSV);
    split(Src_HSV,channels);

   
    Mat H=channels.at(0);
    cout<<"H="<<(int)H.at<uchar>(50,50)<<endl;
    Mat S=channels.at(1);
    cout<<"S="<<(int)S.at<uchar>(50,50)<<endl;
    Mat V=channels.at(2);
    cout<<"V="<<(int)V.at<uchar>(50,50)<<"size="<<V.size()<<endl;
    Mat V0(H.size(),CV_8U,Scalar::all(150));
    //nomalization 
    channels.at(0)=H*150/V;
    
    channels.at(1)=S*150/V;
    channels.at(2)=V0;
    cout<<"V0="<<V0.size()<<endl;
    //merge(channels,Src_HSV);
    //cvtColor(Src_HSV,timg, CV_HSV2BGR);
   */
	cout<<"find squares src.chan="<<src.channels()<<endl;
	cvtColor( src, gray0, CV_BGR2GRAY);
        for( int l = 0; l < N; l++ )
        {
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
        }
    if(cand_1.size()>0)
    {
        
        Rect roi(cand_1[0],cand_1[2]);
		
       Mat ROI_image(src,roi); //src(roi).copyTo(ROI_image);
        int rect_1_true=Color_judge(ROI_image,(cand_1[2].x-cand_1[0].x)*(cand_1[2].y-cand_1[0].y));
	cout<<"rect1 true: "<<rect_1_true<<endl;
        if(rect_1_true)
            squares.push_back(cand_1);
        if(cand_2.size()>0)
        {
            //Mat ROI_image_2;
            Rect roi_2(cand_2[0],cand_2[2]);
			 Mat ROI_image_2(src,roi_2);;
            //src(roi_2).copyTo(ROI_image_2);
            int rect_2_true=Color_judge(ROI_image_2,(cand_2[2].x-cand_2[0].x)*(cand_2[2].y-cand_2[0].y));
	    cout<<"rect2 true: "<<rect_2_true<<endl;
            if(rect_2_true)
                squares.push_back(cand_2);
            if(cand_3.size()>0)
            {
                
                Rect roi_3(cand_3[0],cand_3[2]);
				Mat ROI_image_3(src,roi_3);
               // src(roi_3).copyTo(ROI_image_3);
                int rect_3_true=Color_judge(ROI_image_3,(cand_3[2].x-cand_3[0].x)*(cand_3[2].y-cand_3[0].y));
		cout<<"rect3 true: "<<rect_3_true<<endl;
                if(rect_3_true)
                    squares.push_back(cand_3);
                if(out.size()>0)
                    cout<<"too much rect ! "<<endl;
            }
        }
    }
}
static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
#ifdef _SHOW_PHOTO
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);

#endif
    }
    cout<<"squares.size: "<<squares.size()<<endl;
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
        //cout<<"final tl: "<<tl<<endl;
        //cout<<"final tr: "<<tr<<endl;
        //cout<<"final br: "<<br<<endl;
        //cout<<"final bl: "<<bl<<endl;
#ifdef _SHOW_PHOTO
        char str_y[20];
        char str_z[20];
        char str_x[20];
        char str_tz[20];
        char str_ty[20];
        char str_tx[20];
        sprintf(str_y,"%lf",ry);
        sprintf(str_z,"%lf",rz);
        sprintf(str_x,"%lf",rx);
        sprintf(str_tz,"%lf",tz);
        sprintf(str_ty,"%lf",ty);
        sprintf(str_tx,"%lf",tx);
        string pre_str_y="thetay: ";
        string pre_str_z="thetaz: ";
        string pre_str_x="thetax: ";
        string pre_str_tz="tz: ";
        string pre_str_ty="ty: ";
        string pre_str_tx="tx: ";
        string full_y=pre_str_y+str_y;
        string full_z=pre_str_z+str_z;
        string full_x=pre_str_x+str_x;
        string full_tz=pre_str_tz+str_tz;
        string full_ty=pre_str_ty+str_ty;
        string full_tx=pre_str_tx+str_tx;
        putText(image,full_y,Point(30,30),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
        putText(image,full_z,Point(30,70),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
        putText(image,full_x,Point(30,100),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
        putText(image,full_tz,Point(30,130),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,255),2);
        putText(image,full_ty,Point(30,170),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
        putText(image,full_tx,Point(30,200),CV_FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),2);
#endif
    }
#ifdef _SHOW_PHOTO
    imshow(wndname, image);
#endif
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
    cout<<"tvec: "<<tvec<<endl;
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
    cout<<"thetay: "<<thetay<<endl;
    cout<<"thetaz: "<<thetaz<<endl;
    cout<<"thetax: "<<thetax<<endl;
}
int Color_judge(Mat &src,int area)
{
  //judge from the center point: BGR
  Mat dst;
  cout<<"color judges 1 src.chan="<<src.channels()<<endl;
  cvtColor( src, dst, CV_BGR2HSV);
  //src = dst;
  
  int x=src.cols/2;
  int y=src.rows/2;
  cout<<"RGB of Center ("<<y<<","<<x<<")="<<(int)dst.at<Vec3b>(y,x)[0]<<" "<<(int)dst.at<Vec3b>(y,x)[1]<<" "<<(int)dst.at<Vec3b>(y,x)[2]<<endl;
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
    cout<<"sum: "<<sum<<endl;
    cout<<"portion: "<<sum/area<<endl;
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


 RMVideoCapture capture("/dev/video0", 3);
int exp_time = 62;
int gain = 30;
int brightness_ = 10;
int whiteness_ = 86;
int saturation_ = 60;
void on_expTracker(int,void *)
{
  capture.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
void on_gainTracker(int,void *)
{
  capture.setpara(gain,brightness_,whiteness_,saturation_);// cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
void on_brightnessTracker(int,void*)
{
   capture.setpara(gain,brightness_,whiteness_,saturation_);//cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
void on_whitenessTracker(int,void*)
{
  capture.setpara(gain,brightness_,whiteness_,saturation_);// cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
void on_saturationTracker(int,void*)
{
  capture.setpara(gain,brightness_,whiteness_,saturation_);//cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
int main(int argc, char** argv)
{
 /* VideoCapture capture;
  capture.open(0);
  if (!capture.isOpened())
  {
    cout << " failed to open!" << endl;
    return 1;   
  }
  capture.set(CV_CAP_PROP_FRAME_WIDTH, DEFAULT_FRAME_WIDTH);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, DEFAULT_FRAME_HEIGHT);
  set_camera_exposure(BUFFER_EXPOSURE_VALUE);
  Mat frame;
  capture.read(frame);
  capture.read(frame);
  capture.read(frame);
  #ifdef _SHOW_PHOTO
  namedWindow( wndname, 1 );
  #endif 
  */
   
  #ifdef _SHOW_PHOTO
    namedWindow( wndname, 1 );
  #endif 
    //RMVideoCapture capture("/dev/video0", 3);
    capture.setVideoFormat(800, 600, 1);
    // capture.setExposureTime(0, 62);//settings->exposure_time);
   
    //RMVideoCapture cap("/dev/video0", 3);
    createTrackbar("exposure_time",wndname,&::exp_time,100,on_expTracker);
    createTrackbar("gain",wndname,&::gain,100,on_gainTracker);
    createTrackbar("whiteness",wndname,&::whiteness_,100,on_whitenessTracker);
    createTrackbar("brightness_",wndname,&::brightness_,100,on_brightnessTracker);
    createTrackbar("saturation",wndname,&::saturation_,100,on_saturationTracker);
    on_brightnessTracker(0,0);
    on_expTracker(0,0);
    on_gainTracker(0,0);
    on_saturationTracker(0,0);
    on_whitenessTracker(0,0);
   // capture.info();
   // capture.getCurrentSetting();
  //  capture.setpara(4,0,86,60);
   // VideoCapture capture;
   // capture.open(0);
    if(!capture.startStream())
    {
     cout<<"Open Camera failure.\n"; 
     return 1;
    }
    
        Mat frame;
    //capture>>frame;
    //capture>>frame;
    //capture>>frame;
  vector<vector<Point> > squares;
  init();
  uint32_t cnt = 0;
  Rmp_Config(&rmp, 50000);
  Maf_Init(&maf, maf_buf, MAF_BUF_LEN);
  Tri_Init(&tri, 2.5e4);
  //if (argc < 2) {
  //  printf("Please input device name\n");
  //  return 0;
  //}
  //char* device = argv[1];
  const char* device = "/dev/ttyTHS2";
  if (connect_serial(device,115200) == -1)
  {
    printf("serial open error!\n");
    return -1;
  }
  
  MyThread kylibotMsgPullerTread;
  MyThread kylibotMsgPusherTread;
  
  kylibotMsgPullerTread.create(KylinBotMsgPullerThreadFunc, NULL);
  kylibotMsgPusherTread.create(KylinBotMsgPusherThreadFunc, NULL);
  int lostFlag = 0;
  //KylinBotMsgPullerThreadFunc(NULL);
  while ((!exit_flag))//&&(capture.read(frame))) 
  {
    squares.clear();
    double t = (double)getTickCount();
    
    capture >> frame;
    //cout<<"IN"<<endl;
    if (frame.empty())
      continue;
    
    Mat src=frame.clone();
    findSquares(src,frame, squares);
    drawSquares(frame, squares);
    /*****************************/
    /*******send the object position to ARM*********/
    /***********************************************/
    //imshow(wndname, frame);
    int c = waitKey(1);

    t = ((double)getTickCount() - t)/getTickFrequency();
    cout<<"time: "<<t<<" second"<<endl;

    if((char)c == 'q')
      break;
    
    if(squares.size() > 0)
    {
      lostFlag = 0;
    }
    
    
    if (squares.size() == 0) 
    {
      lostFlag++;
      if(lostFlag >= 3)
      {
	lostFlag = 0;
	tx = 0;
	ty = 0;
	tz = 0;
	rx = 0;
	ry = 0;
	rz = 0;
      }
     
    }
    
    /*
    txKylinMsg.cp.x = tx + kylinMsg.cp.x;
    txKylinMsg.cv.x = 1000;
    txKylinMsg.cp.y = tz + kylinMsg.cp.y;
    txKylinMsg.cv.y = 1000;
    txKylinMsg.cp.z = ry + kylinMsg.cp.z;
    txKylinMsg.cv.z = 1000;
    txKylinMsg.gp.e = ty + kylinMsg.gp.e;
    txKylinMsg.gv.e = 1000;
    */
    
    txKylinMsg.cp.x = tx;
    txKylinMsg.cv.x = 1000;
    txKylinMsg.cp.y = tz;
    txKylinMsg.cv.y = 1000;
    txKylinMsg.cp.z = ry;
    txKylinMsg.cv.z = 1000;
    txKylinMsg.gp.e = ty;
    txKylinMsg.gv.e = 0;
    
    if (abs(tx) < 100 && abs(ty) < 100 && abs(tz) < 100) {
      //txKylinMsg.gp.c = 2199;
      //txKylinMsg.gv.c = 4000;
    } else {
      //txKylinMsg.gp.c = 314;
      //txKylinMsg.gv.c = 4000;
    }
    
    usleep(10000);
    //kylinbot_control();
    //PullMsg();
  }
  
  disconnect_serial();
  
  //while (getchar() == '\0');
  return 0;
}

int main__(int argc, char** argv)
{
  int flag = Kylinbot_Connect();
  if (flag == -1) {
    return -1;
  }
  
  while (flag != -1) {
    usleep(10000);
  }
  
  Kylinbot_Disconnect();
  
  return 0;
}


