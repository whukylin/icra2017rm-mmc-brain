#include "main.h"
#include <math.h>
#include "RMVideoCapture.hpp"
#include "Markdetection.h"
#include "CMT.h"
#include "gui.h"

// 滑台上升速度和下降速度(测试阶段，降低下降速度, 全局速度)
#define GRASP_UP_SPEED 1200
#define GRASP_DOWN_SPEED 800

// 抓子夹有盒子时, 滑台上升和下降速度(普遍降低)
#define GRASP_UP_SPEED_HAVE_BOX 600
#define GRASP_DOWN_SPEED_HAVE_BOX 600
#define GRASP_DOWN_SPEED_HAVE_MANY_BOX 400

//TODO: 检测矩形时, 滑台距离最低点的距离
#define DETECT_SQUARE_GRASP_POSITION 70

//TODO: 小车与盒子之间的距离小于多少时 从矩形检测引导小车切换到超声波引导
#define SQUARE_TO_FIXED_ULTRASONIC_DISTANCE 420

//TODO: 判定 fixed 超声波无法打到盒子并向左移动时的阈值
#define LEFT_MOVE_DISTANCE 0

//TODO: 抓住盒子之后, 滑台上升位置
#define GRASP_UP_HAVE_BOX_POSITION 0

// 基地区坐标 axisX axisY
#define AXISX 0
#define AXISY 2000

//TODO: 放置盒子的时候, 每一堆非第一个盒子放置的位置
#define FIXED_ULTRASONIC_2_PUTBOX 130
// 放置盒子的时候, 每一堆第一个盒子放置的位置
#define FIXED_ULTRASONIC_1_PUTBOX 500

// 抓子松开和合拢速度
#define GRASP_OPEN_SPEED 10000
#define GRASP_CLOSE_SPEED 10000

// 放下盒子之后, 先后推, 后退的距离, 速度以及抓子抬高的高度
#define DIRECT_BACK_MOVE_DISTANCE 400
#define DIRECT_BACK_MOVE_SPEED 500
#define DIRECT_BACK_MOVE_GRASP_UP_POSITION 100

// 堆叠盒子时, 小车后退的距离
#define DIRECT_BACK_MOVE_DISTANCE_PUTBOX 550

// 堆叠盒子时, 放盒子的时候, fixed 超声波距离阈值
#define FIXED_ULTRASONIC_PUTBOX2TO1 130

// 最大搬运盒子数量
#define MAX_BOXNUM 8

// TODO:堆叠模式选择:   1 -> 2+2+2+2=8, 2 -> 2+1+2+1+2=8, 3 -> 1+1+1+1+1+1+1+1=8
#define PUTBOX_MODE 1

//判断盒子是否完全进入抓子的模式: 1 -> 光电对管, 2-> 超声波
#define BOX_IN_GRASP_MODE 1
// 使用超声波判断盒子是否完全进入抓子时, 判断阈值
#define CLAW_CLOSE_SONAR_TRIGGER_DISTANCE 22

// 摄像头与小车轴心的固定偏移
#define DIFFCONST 50

// 小车旋转角度
#define ZROTATION90DEG 1572

// 小车运动速度宏定义(分阶段)
// 阶段 1 : 从原点出发, 抓盒子, 直到切换到 fixed 超声波
#define X_SPEED_1 400
#define Y_SPEED_1 1100
#define Z_SPEED_1 1300
// 矩形检测引导小车旋转的速度
#define Z_SPEED_1_VISION 400
// fixed 超声波引导小车前进的速度
#define FIXED_ULTRASONIC_MOVE_SPEED 200
// left right 超声波对准盒子时, 相对位置控制左右移动的距离量以及左右移动的速度
#define LRDISTANCE 100
#define LRSPEED 100
// mobile 超声波引导小车前进时, 小车移动速度
#define MOBILE_ULTRASONIC_MOVE_SPEED 300

// 阶段 2 : 小车抓取到盒子之后, 回原点的速度
#define X_SPEED_2 600
#define Y_SPEED_2 700
#define Z_SPEED_2 1300

// 阶段 3 : 小车拿着盒子, 到达基地区
#define X_SPEED_3 400
#define Y_SPEED_3 1100
#define Z_SPEED_3 1300
// 矩形检测引导小车旋转的速度
#define Z_SPEED_3_VISION 400

// 阶段 4 : 小车放下盒子, 回原点的速度
#define X_SPEED_4 600
#define Y_SPEED_4 700
#define Z_SPEED_4 1300




#define YSPEED 1100 //forward speed
#define XSPEED 400
#define ZSPEED 1300

#define GRASPSPEED 1200


#define COM_PORT 1 //"COM7"
#define BUF_LEN 256
#define TIMEOUT 30
#define FRAME_N 20000
#define ADDSPEED 100


using namespace cv;
using namespace std;
using cmt::CMT;
double missionStartTimeUs = 0;
double missionEndTimeUs = 0;

double currentTimeUs()
{
    return cvGetTickCount() / cvGetTickFrequency();
}

extern double ry, rz, rx;
extern double tx, ty, tz;
extern const char *wndname;
//Create a CMT object
CMT cmt0;
RMVideoCapture capture("/dev/video0", 3);
//VideoCapture capture;
int exp_time = 62;
int gain = 30;
int brightness_ = 10;
int whiteness_ = 86;
int saturation_ = 60;

int8_t exit_flag = 0;
int workState = 0;
volatile int8_t detection_mode = 0;
volatile bool useUltrasonic = false;

FIFO_t rx_fifo;
uint8_t rx_buf[2][BUF_LEN];

ZGyroMsg_t zgyroMsg;
KylinMsg_t kylinMsg;
Sr04sMsg_t sr04sMsg;
PosCalibMsg_t posCalibMsg;

#define SR04_NUM 4
#define SR04_IDX_F 0
#define SR04_IDX_M 1
#define SR04_IDX_L 2
#define SR04_IDX_R 3

#define SR04_MAF_LEN 5
Maf_t sr04maf[SR04_NUM];
float sr04buf[SR04_NUM][SR04_MAF_LEN];

FIFO_t tx_fifo;
uint8_t tx_buf[2][BUF_LEN];
KylinMsg_t txKylinMsg;

volatile int coutLogicFlag = 0;
int lineflag = 0;
volatile bool finishAbsoluteMoveFlag = false;    //完成绝对位置移动
volatile bool finishDetectBoxFlag = false;       //完成检测盒子(小车到了检测不到盒子的位置)
volatile bool finishDetectCentroidFlag = false;  //完成质心检测
volatile bool finishMobleUltrasonicFlag = false; //超声波到达极限距离
volatile bool finishGraspFlag = false;           //抓子是否合拢
volatile bool finishSlidFlag = false;            //滑台是否达到指定高度
volatile bool finishGraspOpFlag = false;         //抓子是否合拢
volatile bool finishSlidBwFlag = false;          //滑台是否达到指定高度
volatile bool finishAbsoluteMoveFlag_Put = false;
volatile bool finishFixedUltrasonicFlag = false;
volatile bool finish_LR_UltrasonicFlag = false;
volatile bool finish_L_UltrasonicFlag = false;
volatile bool finish_R_UltrasonicFlag = false;
volatile bool finishAbsoluteMoveFlag_BackOrigin = false;

volatile bool finishDetectBoxFlag_PutBox = false;
volatile bool finishFixedUltrasonicFlag_1_PutBox = false;
volatile bool finish_LR_UltrasonicFlag_PutBox = false;
volatile bool finishFixedUltrasonicFlag_2_PutBox = false;
volatile bool finishSlidTpFlag_PutBox = false;
volatile bool finishGraspBwFlag_PutBox = false;

volatile bool finishBackMoveFlag_PutBox2toBox1 = false; // sr04maf[SR04_IDX_F].avg > 550 => true
volatile bool finishGraspBwFlag_PutBox2toBox1 = false;
volatile bool finish_LR_UltrasonicFlag_PutBox2toBox1 = false;
volatile bool finishDetectCentroidFlag_PutBox2toBox1 = false;
volatile bool finishMobleUltrasonicFlag_PutBox2toBox1 = false;
volatile bool finishGraspFlag_PutBox2toBox1 = false;
volatile bool finishSlidFlag_PutBox2toBox1 = false;
volatile bool finishFixedUltrasonicFlag_PutBox2toBox1 = false;
volatile bool finishGraspOpFlag_PutBox2toBox1 = false;
volatile bool finish_PutBox2toBox1_3to2 = false;
volatile bool finish_PutBox2toBox1_2to1 = false;
volatile bool finishBackMoveFlag = false;
volatile int GraspBwCout = 0;
volatile int GraspTpCout = 0;
volatile int absoluteDistanceCout = 0;
volatile bool finishGraspBeforeCentroidFlag = false;

int putBoxNum = 1;
int boxNum = 1; //the num of the box
int GraspTp;
int GraspBw;
int GraspOp;
int GraspCl;
volatile double coutLogicFlag_PutBox = 0;
volatile double coutLogicFlag_PutBox2toBox1 = 0;
void videoMove_PutBox2toBox1();
int moveDistance = 0;
int numDelay = 100000000;
float coutAngle = 0;
static void Dnl_ProcZGyroMsg(const ZGyroMsg_t *zgyroMsg)
{
    //printf("*************************************ZGYRO********************************************\n");
    printf("id=%d,angle=%d,rate=%d\n", zgyroMsg->frame_id, zgyroMsg->angle, zgyroMsg->rate);
}

static void Dnl_ProcKylinMsg(const KylinMsg_t *kylinMsg)
{

    //printf("*************************************KYLIN********************************************\n");
    printf("id=%d,fs=%x,px=%d,py=%d,pz=%d,pe=%d,pc=%d,vx=%d,vy=%d,vz=%d,ve=%d,vc=%d\n", kylinMsg->frame_id, kylinMsg->cbus.fs,
           kylinMsg->cbus.cp.x, kylinMsg->cbus.cp.y, kylinMsg->cbus.cp.z, kylinMsg->cbus.gp.e, kylinMsg->cbus.gp.c,
           kylinMsg->cbus.cv.x, kylinMsg->cbus.cv.y, kylinMsg->cbus.cv.z, kylinMsg->cbus.gv.e, kylinMsg->cbus.gv.c);
}
//超声波信息
static void Dnl_ProcSr04sMsg(const Sr04sMsg_t *sr04sMsg)
{
    //printf("*************************************KYLIN********************************************\n");
    printf("id=%d,fixed=%d,moble=%d,left=%d,right=%d\n", sr04sMsg->frame_id, sr04sMsg->fixed, sr04sMsg->moble, sr04sMsg->left, sr04sMsg->right);
}
//抓子初始化
static void Dnl_ProcPosCalibMsg(const PosCalibMsg_t *posCalibMsg)
{

    //printf("*************************************KYLIN********************************************\n");
    printf("id=%d,el=%d,eh=%d,cl=%d,ch=%d\n", posCalibMsg->frame_id, posCalibMsg->data.el, posCalibMsg->data.eh, posCalibMsg->data.cl, posCalibMsg->data.ch);
}

void PullMsg()
{
    // Get fifo free space
    int len = FIFO_GetFree(&rx_fifo);
    // If fifo free space insufficient, pop one element out
    if (!len)
    {
        uint8_t b;
        len = FIFO_Pop(&rx_fifo, &b, 1);
    }
    // Read input stream according to the fifo free space left
    len = read_serial(rx_buf[1], len, TIMEOUT);
    // Push stream into fifo
    FIFO_Push(&rx_fifo, rx_buf[1], len);
    // Check if any message received
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_kylin, &kylinMsg))
    {
        // Dnl_ProcKylinMsg(&kylinMsg);
    }
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_sr04s, &sr04sMsg))
    {
        Maf_Proc(&sr04maf[SR04_IDX_F], sr04sMsg.fixed);
        Maf_Proc(&sr04maf[SR04_IDX_M], sr04sMsg.moble);
        Maf_Proc(&sr04maf[SR04_IDX_L], sr04sMsg.left);
        Maf_Proc(&sr04maf[SR04_IDX_R], sr04sMsg.right);
        //Dnl_ProcSr04sMsg(&sr04sMsg);
    }
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_zgyro, &zgyroMsg))
    {
        //Dnl_ProcZGyroMsg(&zgyroMsg);
    }
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_pos_calib, &posCalibMsg))
    {
        //Dnl_ProcPosCalibMsg(&posCalibMsg);
    }
}

void *KylinBotMsgPullerThreadFunc(void *param)
{
    while (exit_flag == 0)
    {
        PullMsg();
        usleep(1000);
    }
    return NULL;
}

void PushMsg()
{

    uint32_t len = Msg_Push(&tx_fifo, tx_buf[1], &msg_head_kylin, &txKylinMsg);
    FIFO_Pop(&tx_fifo, tx_buf[1], len);
    write_serial(tx_buf[1], len, TIMEOUT);
}

void *KylinBotMsgPusherThreadFunc(void *param)
{

    while (exit_flag == 0)
    {
        PushMsg();
        usleep(4000);
    }
}

void init()
{
    uint32_t i = 0;
    FIFO_Init(&rx_fifo, rx_buf[0], BUF_LEN);
    FIFO_Init(&tx_fifo, tx_buf[0], BUF_LEN);
    for (i = 0; i < SR04_NUM; i++)
    {
        Maf_Init(&sr04maf[i], sr04buf[i], SR04_MAF_LEN);
    }
}

typedef struct
{

    uint8_t dir;
    uint32_t cnt;
    uint32_t scl;
} Tri_t;

void Tri_Init(Tri_t *tri, uint32_t scl)
{

    tri->scl = scl;
    tri->cnt = 0;
    tri->dir = 0;
}

float Tri_Proc(Tri_t *tri)
{
    if (tri->dir == 0)
    {
        if (tri->cnt < tri->scl)
        {
            tri->cnt++;
        }
        else
        {
            tri->dir = 1;
        }
    }
    else if (tri->dir == 1)
    {
        if (tri->cnt > 0)
        {
            tri->cnt--;
        }
    }
    return (float)tri->cnt / (float)tri->scl;
}

void Tri_Reset(Tri_t *tri)
{

    tri->cnt = 0;
    tri->dir = 0;
    //memset(tri, 0, sizeof(Tri_t));
}

#define RMP_CNT 220000
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

    if (cnt < 5e4)
    {
        if (dir != 1)
        {
            Rmp_Reset(&rmp);
            Tri_Reset(&tri);
            dir = 1;
        }
        // Move forward
        printf("Move forward\n");
        memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
        txKylinMsg.cbus.cp.y = 1000;
        txKylinMsg.cbus.cv.y = 1000 * Tri_Proc(&tri); //* Rmp_Calc(&rmp);
    }
    else if (cnt < 10e4)
    {
        if (dir != 2)
        {
            Rmp_Reset(&rmp);
            Tri_Reset(&tri);
            dir = 2;
        }
        // Move right
        printf("Move right\n");
        memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
        txKylinMsg.cbus.cp.x = 1000;
        txKylinMsg.cbus.cv.x = 1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
    }
    else if (cnt < 15e4)
    {
        if (dir != 3)
        {
            Rmp_Reset(&rmp);
            Tri_Reset(&tri);
            dir = 3;
        }
        // Move backward
        printf("Move backward\n");
        memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
        txKylinMsg.cbus.cp.y = -1000;
        txKylinMsg.cbus.cv.y = -1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
    }
    else if (cnt < 20e4)
    {
        if (dir != 4)
        {
            Rmp_Reset(&rmp);
            Tri_Reset(&tri);
            dir = 4;
        }
        // Move left
        printf("Move left\n");
        memset(&txKylinMsg, 0, sizeof(KylinMsg_t));
        txKylinMsg.cbus.cp.x = -1000;
        txKylinMsg.cbus.cv.x = -1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
    }
    else
    {
        cnt = 0;
    }

    printf("cnt=%d\n", tri.cnt);
}

void rstRmp()
{
    Rmp_Config(&rmp, RMP_CNT);
    Rmp_Reset(&rmp);
}

float genRmp()
{
    return Rmp_Calc(&rmp);
}

int CMT_temdetect(Mat frame, int &diff_x, int &diff_y)
{
    Mat im_gray;

    cvtColor(frame, im_gray, CV_BGR2GRAY);

    //Let CMT process the frame
    cmt0.processFrame(im_gray);
    Point2f vertices[4];
    cmt0.bb_rot.points(vertices);
    Point center;
    center.x = (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4;
    center.y = (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4;
    diff_x = center.x - CX;
    diff_y = center.y - CY;
    cout << "diff x: " << diff_x << endl;
    cout << "diff y: " << diff_y << endl;
    for (int i = 0; i < 4; i++)
    {
        line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(255, 0, 0));
    }

    imshow(wndname, frame);
    return 1;
}

void *KylinBotMarkDetecThreadFunc(void *param)
{
    Mat frame;
    vector<vector<Point>> squares;
    int lostFlag = false;
    //KylinBotMsgPullerThreadFunc(NULL);
    //printf("hjhklhjllllllhkl\n");
    printf("exit_flag=%d\n", exit_flag);
    int lostCount = 0;
    int CountVframe = 0;
    int fflage = -1;
    while (exit_flag == 0) //&&(capture.read(frame)))
    {
        squares.clear();
        double t = (double)getTickCount();

        capture >> frame;
        if (frame.empty())
            continue;

        int dif_x = 0, dif_y = 0;
        Mat src = frame.clone();
        //cout << "detection_mode=" << (int)detection_mode << endl;

        cout << "sr04maf[SR04_IDX_L].avg:" << sr04maf[SR04_IDX_L].avg << " sr04maf[SR04_IDX_R].avg: " << sr04maf[SR04_IDX_R].avg << " ";
        cout << " sr04maf[SR04_IDX_F].avg:" << sr04maf[SR04_IDX_F].avg << " sr04maf[SR04_IDX_M].avg: " << sr04maf[SR04_IDX_M].avg << endl;
        //cout << "finishDetectCentroidFlag" << endl;
        //cout << "GraspBwCout: " << GraspBwCout << " GraspTpCout: " << GraspTpCout << endl;
        //cout<<"GraspOp:"<<GraspOp<<"GraspCl"<<GraspCl<<endl;
        //cout << "kylinMsg.cbus.gp.e" << kylinMsg.cbus.gp.e << "  " << kylinMsg.cbus.gp.c << endl;
        //cout << "finishDetectBoxFlag_PutBox: " << finishDetectBoxFlag_PutBox << endl;
        cout << "coutLogicFlag: " << coutLogicFlag << " coutLogicFlag_PutBox: " << coutLogicFlag_PutBox << " coutLogicFlag_PutBox2toBox1: " << coutLogicFlag_PutBox2toBox1 << endl;
        cout << "absoluteDistanceCout: " << absoluteDistanceCout << endl;


        
	cout << "fflage: "<<fflage<<" tx:"<<tx<<" Vframe:"<<CountVframe<<endl;
        detection_mode=1; //for testing        
        cout << "Grasp: " << kylinMsg.cbus.gp.c << " SwitchFlag: " << kylinMsg.cbus.fs << endl;

	switch (detection_mode)
		
	{
		case 0: //do nothing
			//TODO:
			//imshow("IM",frame);
			//cout << "detection_mode=" << (int)detection_mode << endl;
			break;
		case 1: //detect squares
			//cout << "detection_mode=" << (int)detection_mode << endl;
			findSquares(src, frame, squares);
			LocationMarkes(squares);
			drawSquares(frame, squares);
			if (squares.size() > 0)
			{
				lostCount = 0;
				lostFlag = false;
				CountVframe++;
				// txKylinMsg.cbus.cp.x = tx;
				// txKylinMsg.cbus.cv.x = 500;
				// txKylinMsg.cbus.cp.y = tz;
				// txKylinMsg.cbus.cv.y = 800;
				// txKylinMsg.cbus.cp.z = ry * 3141.592654f / 180;
				// txKylinMsg.cbus.cv.z = 500;
				// txKylinMsg.cbus.gp.e = ty;
				// txKylinMsg.cbus.gv.e = 0;
			}
			if (squares.size() == 0)
			{
				lostCount++;
				if (lostCount >= 3)
				{
					printf("lost frame\n");
					lostCount = 0;
					lostFlag = true;
					tx = DIFFCONST;
					ty = 0;
					tz = 0;
					rx = 0;
					ry = 0;
					rz = 0;
					rstRmp();
				}
			}
            //TODO: 本 if 语句使用 fixed 还是 mobile 超声波?
            //TODO: 矩形检测 flag 置 true 过程中, 超声波阈值宏定义
			if (sr04maf[SR04_IDX_M].avg < 500 || (abs(tz) < 700 && (lostFlag == false) && CountVframe > 10))
			{ //Usue ultra sonic distance for controlling. Detection_mode will be changed in main.
				finishDetectBoxFlag = true;
				
				CountVframe = 0;
			}
			else
			{
				finishDetectBoxFlag = false;
			}
			if (coutLogicFlag == 9 && (sr04maf[SR04_IDX_F].avg < 500 || (abs(tz) < 500 && (lostFlag == false) && CountVframe > 10)))
			{
				finishDetectBoxFlag_PutBox = true;
				CountVframe = 0;
			}
			else
			{
				finishDetectBoxFlag_PutBox = false;
			}
			printf("tz=%lf\n", tz);
			break;
			case 2: //detect green area
				//cout << "detection_mode=" << (int)detection_mode << endl;
				
				fflage = CMT_temdetect(src, dif_x, dif_y);
				if (fflage == 0)
					CountVframe++;
				tx = 2 * (dif_x - DIF_CEN);
				// txKylinMsg.cbus.cp.x = 10 * dif_x;
				cout << "tx=" << tx << endl;
				if (abs(tx) < 30 && (CountVframe > 100 || fflage)) //number of pixels
				{
					CountVframe = 0;
					finishDetectCentroidFlag = true;
					if (coutLogicFlag == INT_MAX && finish_LR_UltrasonicFlag_PutBox2toBox1 == true)
					{
						finishDetectCentroidFlag_PutBox2toBox1 = true;
					}
				}
				break;
			case 3: //follow line
				//cout << "detection_mode=" << (int)detection_mode << endl;
				break;
			case 4: //follow line
				//cout << "detection_mode=" << (int)detection_mode << endl;
				break;
			case 5: //follow line
				//cout << "detection_mode=" << (int)detection_mode << endl;
				
				//TODO:
				break;
			default:
				break;
	}
	int c = waitKey(1);
        if ((char)c == 'q')
            break;
    }
}

void on_expTracker(int, void *)
{
    //capture.set(CV_CAP_PROP_EXPOSURE,exp_time);
    capture.setExposureTime(0, ::exp_time); //settings->exposure_time);
}
void on_gainTracker(int, void *)
{
    //capture.set(CV_CAP_PROP_GAIN,::gain);
    capture.setpara(gain, brightness_, whiteness_, saturation_); // cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
void on_brightnessTracker(int, void *)
{

    //capture.set(CV_CAP_PROP_BRIGHTNESS,::brightness_);
    capture.setpara(gain, brightness_, whiteness_, saturation_); //cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
void on_whitenessTracker(int, void *)
{
    //capture.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U,::whiteness_);
    capture.setpara(gain, brightness_, whiteness_, saturation_); // cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
void on_saturationTracker(int, void *)
{
    //capture.set(CV_CAP_PROP_SATURATION,::saturation_);
    capture.setpara(gain, brightness_, whiteness_, saturation_); //cap.setExposureTime(0, ::exp_time);//settings->exposure_time);
}
bool setcamera()
{
#ifdef _SHOW_PHOTO
    namedWindow(wndname, 1);
#endif
    //RMVideoCapture capture("/dev/video0", 3);
    capture.setVideoFormat(800, 600, 1);

    //capture.open(0);
    //capture.set(CV_CAP_PROP_FRAME_WIDTH,800);
    //capture.set(CV_CAP_PROP_FRAME_HEIGHT,600);
    // capture.setExposureTime(0, 62);//settings->exposure_time);
    if (!capture.startStream())
    {
        cout << "Open Camera failure.\n";
        return false;
    }
    Mat img;
    //printf("%d %d \n",img.cols,img.rows);
    //if(img.empty())
    //	return 0;
    capture >> img;
    cout << img.cols << " " << img.rows << endl;
    namedWindow("Tuning", 1);
    createTrackbar("exposure_time", "Tuning", &::exp_time, 100, on_expTracker);
    createTrackbar("gain", "Tuning", &::gain, 100, on_gainTracker);
    createTrackbar("whiteness", "Tuning", &::whiteness_, 100, on_whitenessTracker);
    createTrackbar("brightness_", "Tuning", &::brightness_, 100, on_brightnessTracker);
    createTrackbar("saturation", "Tuning", &::saturation_, 100, on_saturationTracker);
    on_brightnessTracker(0, 0);
    on_expTracker(0, 0);
    on_gainTracker(0, 0);
    on_saturationTracker(0, 0);
    on_whitenessTracker(0, 0);

    while (1)
    {
        capture >> img;

        if (img.empty())
            continue;
        imshow("Tuning", img);
        int c = waitKey(20);
        if ((char)c == 'w')
            break;
    }
    destroyWindow("Tuning");
    usleep(1000);
    return true;
    //RMVideoCapture cap("/dev/video0", 3);
}
void logicInit()
{
    txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
                                       //detection_mode = 0;                //摄像头关闭
                                       //cout << "Logic init finish!!" << endl;
}

KylinMsg_t kylinOdomCalib;
static uint32_t frame_cnt = 0;
uint8_t updateOdomCalib()
{
    uint32_t frame_id = kylinMsg.frame_id;
    while (frame_cnt < 50)
    {
        if (kylinMsg.frame_id != frame_id)
        {
            frame_id = kylinMsg.frame_id;
            frame_cnt++;
        }
        //cout << "waiting for new KylinMsg" << endl;
    }
    memcpy(&kylinOdomCalib, &kylinMsg, sizeof(KylinMsg_t));
    //cout << "kylinOdomCalib updated!" << endl;
    return 1;
}

void calibPx()
{
    kylinOdomCalib.cbus.cp.x = kylinMsg.cbus.cp.x;
}

void calibPy()
{
    kylinOdomCalib.cbus.cp.y = kylinMsg.cbus.cp.y;
}

void calibPz()
{
    kylinOdomCalib.cbus.cp.z = kylinMsg.cbus.cp.z;
}

void calibPz90()
{
    kylinOdomCalib.cbus.cp.z = kylinMsg.cbus.cp.z + ZROTATION90DEG;
}

KylinMsg_t kylinOdomError;
uint8_t updateOdomError()
{
    kylinOdomError.frame_id = kylinMsg.frame_id;
    kylinOdomError.cbus.cp.x = txKylinMsg.cbus.cp.x - kylinMsg.cbus.cp.x; // + kylinOdomCalib.cbus.cp.x;
    kylinOdomError.cbus.cp.y = txKylinMsg.cbus.cp.y - kylinMsg.cbus.cp.y; // + kylinOdomCalib.cbus.cp.y;
    kylinOdomError.cbus.cp.z = txKylinMsg.cbus.cp.z - kylinMsg.cbus.cp.z; // + kylinOdomCalib.cbus.cp.z;
    //cout << "ref.x=" << txKylinMsg.cbus.cp.x << ", fdb.x=" << kylinMsg.cbus.cp.x << endl;
    //cout << "ref.y=" << txKylinMsg.cbus.cp.y << ", fdb.y=" << kylinMsg.cbus.cp.y << endl;
    kylinOdomError.cbus.gp.e = txKylinMsg.cbus.gp.e - kylinMsg.cbus.gp.e; // + kylinOdomCalib.cbus.gp.e;
    kylinOdomError.cbus.gp.c = txKylinMsg.cbus.gp.c - kylinMsg.cbus.gp.c; // + kylinOdomCalib.cbus.gp.c;
    kylinOdomError.cbus.cv.x = txKylinMsg.cbus.cv.x - kylinMsg.cbus.cp.x; // + kylinOdomCalib.cbus.cv.x;
    kylinOdomError.cbus.cv.y = txKylinMsg.cbus.cv.y - kylinMsg.cbus.cv.y; // + kylinOdomCalib.cbus.cv.y;
    kylinOdomError.cbus.cv.z = txKylinMsg.cbus.cv.z - kylinMsg.cbus.cv.z; // + kylinOdomCalib.cbus.cv.z;
    kylinOdomError.cbus.gv.e = txKylinMsg.cbus.gv.e - kylinMsg.cbus.gv.e; // + kylinOdomCalib.cbus.gv.e;
    kylinOdomError.cbus.gv.c = txKylinMsg.cbus.gv.c - kylinMsg.cbus.gv.c; // + kylinOdomCalib.cbus.gv.c;
}

float ramp = 0;
int lastWs = 0;
void videoMove_PutBox();
/*************************************************************************
*  函数名称：switchFlagFun
*  功能说明：判断盒子是否完全进入抓子
*  参数说明：BOX_IN_GRASP_MODE: 1->光电对管, 2->超声波
*  函数返回：无
*  修改时间：2017-05-17
*************************************************************************/
bool switchFlagFun()
{
    if(BOX_IN_GRASP_MODE == 1)  //使用光电对管
    {
        return (txKylinMsg.cbus.fs & (1u << 2));
    }
    else                        //使用 mobile 超声波
    {
        return (sr04maf[SR04_IDX_M].avg < CLAW_CLOSE_SONAR_TRIGGER_DISTANCE);
    }
    
}

/*************************************************************************
*  函数名称：txKylinMsg_xyz_Fun
*  功能说明：xyz 位置参数传递函数
*  参数说明：cpx cpy cpz 为 x y z 方向的位移(位置) cvx cvy cvz 为 x y z 方向的速度
*  函数返回：无
*  修改时间：2017-05-17
*************************************************************************/
void txKylinMsg_xyz_Fun(int16_t cpx, int16_t cvx, int16_t cpy, int16_t cvy, int16_t cpz, int16_t cvz)
{
    txKylinMsg.cbus.cp.x = cpx;     //x 左右移动
    txKylinMsg.cbus.cv.x = cvx;
    txKylinMsg.cbus.cp.y = cpy;     //y 前后移动
    txKylinMsg.cbus.cv.y = cvy;
    txKylinMsg.cbus.cp.z = cpz;     //转角 
    txKylinMsg.cbus.cv.z = cvz;
}

/*************************************************************************
*  函数名称：txKylinMsg_ec_Fun
*  功能说明：e c 位置参赛传递函数
*  参数说明：cpe cpc 为 e c 方向的位移(位置) cve cvc 为 e c 的速度
*  函数返回：无
*  修改时间：2017-05-17
*************************************************************************/
void txKylinMsg_ec_Fun(int16_t gpe, int16_t gve, int16_t gpc, int16_t gvc)
{
    txKylinMsg.cbus.gp.e = gpe;     //滑台
    txKylinMsg.cbus.gv.e = gve;
    txKylinMsg.cbus.gp.c = gpc;     //抓子 
    txKylinMsg.cbus.gv.c = gvc;
}

/*************************************************************************
*  函数名称：unFirstBoxJudgeFun
*  功能说明：判断当前盒子非第一个盒子
*  参数说明：无
*  函数返回：bool 类型, 直接当作 if 判断语句的判断条件
*  修改时间：2017-05-17
*************************************************************************/
bool unFirstBoxJudgeFun()
{
    if (PUTBOX_MODE == 1) //2+2+2+2
    {
        return (boxNum == 4);
    }
    if (PUTBOX_MODE == 2) //2+1+2+1+2
    {
        return (boxNum != 3);
    }
    if (PUTBOX_MODE == 3) //1+1+1+1+1+1+1+1
    {
        return (boxNum != 4 && boxNum != 7);
    }
}
/*************************************************************************
*  函数名称：firstBoxJudgeFun
*  功能说明：判断当前盒子是第一个盒子
*  参数说明：无
*  函数返回：bool 类型, 直接当作 if 判断语句的判断条件
*  修改时间：2017-05-17
*************************************************************************/
bool firstBoxJudgeFun()
{
    if (PUTBOX_MODE == 1) //2+2+2+2
    {
        return (boxNum == 1 || boxNum != 4);
    }
    if (PUTBOX_MODE == 2) //2+1+2+1+2
    {
        return (boxNum == 1 || boxNum == 3);
    }
    if (PUTBOX_MODE == 3) //1+1+1+1+1+1+1+1
    {
        return (boxNum == 1 || boxNum == 4 || boxNum == 7);
    }
}

int main(int argc, char **argv)
{
    missionStartTimeUs = currentTimeUs();
    if (!setcamera())
    {
        cout << "Setup camera failure. Won't do anything." << endl;
        return -1;
    }

    init();
    rstRmp();
    //uint32_t cnt = 0;
    //Rmp_Config(&rmp, 50000);
    Maf_Init(&maf, maf_buf, MAF_BUF_LEN);
    Tri_Init(&tri, 2.5e4);

    const char *device = "/dev/ttyTHS2";
    if (connect_serial(device, 115200) == -1)
    {
        printf("serial open error!\n");
        return -1;
    }

    //set CMT
    Mat im0_src;
    //Initialization bounding box
    Rect rect;
    im0_src = imread("../arrow/arrow1.jpg");
    if (im0_src.empty())
    {
        cout << "load template image error" << endl;
        return 0;
    }
    rect = Rect(362, 201, 110, 135); //TODO: Justify
    //Convert im0 to grayscale
    Mat im0_gray;
    cvtColor(im0_src, im0_gray, CV_BGR2GRAY);
    //Initialize CMT
    cmt0.initialize(im0_gray, rect);

    MyThread kylibotMsgPullerTread;
    MyThread kylibotMsgPusherTread;
    MyThread kylibotMarkDetectionTread;

    kylibotMsgPullerTread.create(KylinBotMsgPullerThreadFunc, NULL);
    kylibotMsgPusherTread.create(KylinBotMsgPusherThreadFunc, NULL);
    kylibotMarkDetectionTread.create(KylinBotMarkDetecThreadFunc, NULL);

    /*Flag变量汇总:
     *    finishAbsoluteMoveFlag      完成绝对位置移动
     *    finishDetectBoxFlag         完成检测盒子(小车到了检测不到盒子的位置)
     *    finishDetectCentroidFlag    完成质心检测
     *    finishMobleUltrasonicFlag   超声波到达极限距离
     *    finishGraspFlag             抓子是否合拢
     *    finishSlidFlag              滑台是否达到指定高度
     */
    updateOdomCalib();
    logicInit(); //逻辑控制初始化

    GraspTp = posCalibMsg.data.el;
    GraspBw = posCalibMsg.data.eh;
    GraspOp = posCalibMsg.data.cl;
    GraspCl = posCalibMsg.data.ch;

    GraspBwCout = GraspBw;
    GraspTpCout = GraspTp;
    double absoluteDistance = 100;
    double absuluteAngle = 100;
    double absuluteGraspOpCl = 100;
    double absuluteGrasp = 100;
    int workState0_Num = 0, workState1_Num = 0, workState2_Num = 0, workState3_Num = 0, workState4_Num = 0;

    while ((!exit_flag)) //&&(capture.read(frame)))
    {
        updateOdomError();
        //如果当前处于绝对位置控制模式,进入判断条件
        if (txKylinMsg.cbus.fs & (1u << 30)) //0xFF == 1111 1111   0x80000000
        {

            absoluteDistance = pow(pow((kylinOdomError.cbus.cp.x), 2) + pow((kylinOdomError.cbus.cp.y), 2), 0.5);
            absoluteDistanceCout = absoluteDistance;
            absuluteAngle = abs(kylinOdomError.cbus.cp.z);
            absuluteGrasp = abs(kylinOdomError.cbus.gp.e);
            absuluteGraspOpCl = abs(kylinOdomError.cbus.gp.c);
            if (coutLogicFlag != 12 && absoluteDistance < 20 && absuluteAngle < 5.0f * PI / 2.0f) // && absuluteGrasp < 10)
            {
                finishAbsoluteMoveFlag = true;
            }
            if (coutLogicFlag == 10 && absuluteGrasp < 10) //(GraspTp + GBw) / 2.f)
            //if(coutLogicFlag == 7 && abs(txKylinMsg.cbus.gp.e - kylinMsg.cbus.gp.e) < 20)
            {
                finishSlidBwFlag = true;
            }
            if (coutLogicFlag == 11 && absuluteGraspOpCl < 250) //4.6 bugs 250 about sliding table
            {
                finishGraspOpFlag = true;
            }
            if (boxNum == 1 && coutLogicFlag == 9 && absoluteDistance < 10) //&& absuluteAngle < 5.0f * PI / 2.0f)
            {
                finishAbsoluteMoveFlag_Put = true;
            }
            if (coutLogicFlag == 12 && absoluteDistance < 10 && absuluteAngle < 5.0f * PI / 2.0f) // && absuluteGrasp < 10)
            {
                finishAbsoluteMoveFlag_BackOrigin = true;
            }
        }

        if ((txKylinMsg.cbus.fs & (1u << 30)) == 0x00000000)
        {
            /*************************************************************************
            *
            *  复用代码
            *
            *************************************************************************/
            //进行左右超声波对准的时候, 复用此段代码
            if (sr04maf[SR04_IDX_L].avg > 300 && sr04maf[SR04_IDX_R].avg < 300)
            {
                moveDistance = LRDISTANCE;
            }
            if (sr04maf[SR04_IDX_L].avg < 300 && sr04maf[SR04_IDX_R].avg > 300)
            {
                moveDistance = -(LRDISTANCE + 30);
            }

            /*************************************************************************
            *
            *  抓取盒子阶段
            *
            *************************************************************************/
            // 抓取盒子阶段, 矩形检测引导切换到超声波引导 (已完成, 未测试)
            if (sr04maf[SR04_IDX_F].avg < SQUARE_TO_FIXED_ULTRASONIC_DISTANCE && finishDetectBoxFlag == true)
            {
                finishFixedUltrasonicFlag = true;
            }

            // 抓取盒子阶段, 左右超声波对准标志位
            if (coutLogicFlag == 3 && sr04maf[SR04_IDX_L].avg > 300 && sr04maf[SR04_IDX_R].avg > 300)
            {
                finish_LR_UltrasonicFlag = true;
                moveDistance = 0;
            }

            //抓取盒子阶段, 完成左右超声波对准, 在进行质心对准之前, 现将滑台升高的标志位
            if (finish_LR_UltrasonicFlag == true && kylinMsg.cbus.gp.e <= GraspBw - DETECT_SQUARE_GRASP_POSITION)
            {
                finishGraspBeforeCentroidFlag = true;
            }

            //抓取盒子阶段, mobile超声波引导小车抓盒子标志位
            if (switchFlagFun()) //if (sr04maf[SR04_IDX_M].avg < CLAW_CLOSE_SONAR_TRIGGER_DISTANCE)
            {
                finishMobleUltrasonicFlag = true;
            }
            //抓取盒子阶段, 抓子合拢标志位
            if (kylinMsg.cbus.gp.c == GraspCl)
            {
                finishGraspFlag = true;
            }
            //Graps OpenfinishAbsoluteMoveFlag
            //抓取盒子阶段, 滑台上升到滑台中心标志位
            if (coutLogicFlag == 7 && kylinMsg.cbus.gp.e <= (GraspBw + GraspTp) / 2.0 + 50) //(GraspTp + GraspBw) / 2.f)
            //if(coutLogicFlag == 7 && abs(txKylinMsg.cbus.gp.e - kylinMsg.cbus.gp.e) < 20)
            {
                finishSlidFlag = true;
            }

            /*************************************************************************
            *
            *  放置盒子阶段
            *
            *************************************************************************/
            //放置盒子阶段, fixed 超声波引导小车前进标志位 (每一堆的第一个盒子)
            if (coutLogicFlag == 9 && sr04maf[SR04_IDX_F].avg < FIXED_ULTRASONIC_1_PUTBOX && finishDetectBoxFlag_PutBox == true)
            {
                finishFixedUltrasonicFlag_1_PutBox = true;
            }
            //放置盒子阶段, 在进行左右超声波对准之前, 首先将抓子降到最低点
            if (coutLogicFlag == 9 && finishFixedUltrasonicFlag_1_PutBox == true && kylinMsg.cbus.gp.e >= GraspBw - 60)
            {
                finishGraspBwFlag_PutBox = true;
            }
            //放置盒子阶段, 左右超声波对准的标志位
            if (coutLogicFlag == 9 && sr04maf[SR04_IDX_L].avg > 350 && sr04maf[SR04_IDX_R].avg > 350 && finishGraspBwFlag_PutBox == true)
            {
                finish_LR_UltrasonicFlag_PutBox = true;
                moveDistance = 0;
            }
            //放置盒子阶段, 判断盒子放置的位置的标志位, 模式不同, 判断条件不同
            //TODO: 放盒子的时候, 高度宏定义
            if (PUTBOX_MODE == 1) //2+2+2+2
            {
                if (boxNum != 4)
                {
                    if (coutLogicFlag == 9 && kylinMsg.cbus.gp.e <= GraspBw - 15 && finish_LR_UltrasonicFlag_PutBox == true)
                    {
                        finishSlidTpFlag_PutBox = true;
                    }
                }
                else
                {
                    if (coutLogicFlag == 9 && kylinMsg.cbus.gp.e <= GraspBw - 15 - 420 && finish_LR_UltrasonicFlag_PutBox == true)
                    {
                        finishSlidTpFlag_PutBox = true;
                    }
                }
            }
            if (PUTBOX_MODE == 2)   //2+1+2+1+2
            {
                if (boxNum == 1 || boxNum == 3)
                {
                    if (coutLogicFlag == 9 && kylinMsg.cbus.gp.e <= GraspBw - 15 && finish_LR_UltrasonicFlag_PutBox == true)
                    {
                        finishSlidTpFlag_PutBox = true;
                    }
                }
                else if (boxNum == 2 || boxNum == 4)
                {
                    if (coutLogicFlag == 9 && kylinMsg.cbus.gp.e <= GraspBw - 15 - 420 && finish_LR_UltrasonicFlag_PutBox == true)
                    {
                        finishSlidTpFlag_PutBox = true;
                    }
                }
                else if (boxNum == 5)
                {
                    if (coutLogicFlag == 9 && kylinMsg.cbus.gp.e <= GraspBw - 15 - 620 && finish_LR_UltrasonicFlag_PutBox == true)
                    {
                        finishSlidTpFlag_PutBox = true;
                    }
                }
            }
            if (PUTBOX_MODE == 3)   //1+1+1+1+1+1+1+1
            {
                if (coutLogicFlag == 9 && kylinMsg.cbus.gp.e <= GraspBw - 15 - 210 * ((boxNum - 1) % 3) && finish_LR_UltrasonicFlag_PutBox == true)
                {
                    finishSlidTpFlag_PutBox = true;
                }
            }

            //放置盒子阶段, 因为要堆几堆, 每一堆的非第一个盒子放置的位置
            if (coutLogicFlag == 9 && sr04maf[SR04_IDX_F].avg < FIXED_ULTRASONIC_2_PUTBOX && finishSlidTpFlag_PutBox == true)
            {
                finishFixedUltrasonicFlag_2_PutBox = true;
            }

            //放置盒子阶段, 完成放置任务, 小车直接后退的完成标志位
            if (workState == 4 && sr04maf[SR04_IDX_F].avg > DIRECT_BACK_MOVE_DISTANCE)
            {
                finishBackMoveFlag = true;
            }

            /*************************************************************************
            *
            *  堆叠盒子阶段
            *
            *************************************************************************/
            //TODO: 当前只能进行三堆的堆叠, 以后修改成任意堆, 便于现场应变
            if (((coutLogicFlag == INT_MAX - 1) && putBoxNum == 1) || ((coutLogicFlag == INT_MAX) && putBoxNum == 2))
            {
                //放下所有盒子之后, 先后退, 后退的完成的标志位
                if (sr04maf[SR04_IDX_F].avg > DIRECT_BACK_MOVE_DISTANCE_PUTBOX && coutLogicFlag_PutBox2toBox1 == 10.1)
                {
                    finishBackMoveFlag_PutBox2toBox1 = true;
                }
                //超声波对准之前, 先将抓子降到最低点的标志位
                if (kylinMsg.cbus.gp.e >= GraspBw - 10 && finishBackMoveFlag_PutBox2toBox1 == true)
                {
                    finishGraspBwFlag_PutBox2toBox1 = true;
                }
                //超声波左右对准
                if (sr04maf[SR04_IDX_L].avg > 300 && sr04maf[SR04_IDX_R].avg > 300 && finishGraspBwFlag_PutBox2toBox1 == true)
                {
                    finish_LR_UltrasonicFlag_PutBox2toBox1 = true;
                    moveDistance = 0;
                }
                // 盒子完全进入抓子标志位
                if (switchFlagFun() && finish_LR_UltrasonicFlag_PutBox2toBox1 == true) //if (sr04maf[SR04_IDX_M].avg < CLAW_CLOSE_SONAR_TRIGGER_DISTANCE && finish_LR_UltrasonicFlag_PutBox2toBox1 == true)
                {
                    finishMobleUltrasonicFlag_PutBox2toBox1 = true;
                }
                // 抓子完全合拢标志位
                if (kylinMsg.cbus.gp.c == GraspCl && finishMobleUltrasonicFlag_PutBox2toBox1 == true)
                {
                    finishGraspFlag_PutBox2toBox1 = true;
                }
                // 将盒子抬到最高点
                // TODO: 抬升高度宏定义
                if (kylinMsg.cbus.gp.e <= GraspTp + 5 && finishGraspFlag_PutBox2toBox1 == true)
                {
                    finishSlidFlag_PutBox2toBox1 = true;
                }
                // 当fixed超声波距离小于多少时, 放下盒子
                if (sr04maf[SR04_IDX_F].avg < FIXED_ULTRASONIC_PUTBOX2TO1 && finishSlidFlag_PutBox2toBox1 == true)
                {
                    finishFixedUltrasonicFlag_PutBox2toBox1 = true;
                }
                // 松开抓子
                if (kylinMsg.cbus.gp.c == GraspOp && finishFixedUltrasonicFlag_PutBox2toBox1 == true)
                {
                    finishGraspOpFlag_PutBox2toBox1 = true;
                }
            }
        }

        //workState = 10;
        //coutLogicFlag = INT_MAX;
        //videoMove_PutBox2toBox1();

        //boxNum = 6;
        //workStateFlagPrint(); //打印当前状态

        switch (workState)
        {
        case 0:
            coutLogicFlag = 0;
            //关闭视觉
            detection_mode = 0;
            txKylinMsg.cbus.fs |= (1u << 30); //切换到绝对位置控制模式
            //TODO: 小车移动速度宏定义
            //在原点处旋转 90 度
            txKylinMsg_xyz_Fun(kylinOdomCalib.cbus.cp.x, 0, kylinOdomCalib.cbus.cp.y, 0, -ZROTATION90DEG + kylinOdomCalib.cbus.cp.z, Z_SPEED_1 * genRmp());
            //抓子张开
            txKylinMsg_ec_Fun(GraspBw - DETECT_SQUARE_GRASP_POSITION, GRASP_UP_SPEED, GraspOp, GRASP_OPEN_SPEED);
            if (finishAbsoluteMoveFlag == true)
            {
                lastWs = workState;
                rstRmp();
                workState = 1; //完成移动，进入下一阶段
                finishAbsoluteMoveFlag_BackOrigin = false;
                finishAbsoluteMoveFlag = false;
            }
            break;
        case 1:
            //if (lastWs != 1) rstRmp();
            //矩形检测
            if (finishDetectBoxFlag == false)
            {
                ramp = genRmp();
                coutLogicFlag = 1;
                //打开视觉,检测矩形
                detection_mode = 1;
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

                //视觉引导小车前进, 直到小车与盒子之间的距离小于 TODO: 多少厘米 宏定义
                txKylinMsg_xyz_Fun(tx - DIFFCONST, X_SPEED_1 * ramp, tz, Y_SPEED_1 * ramp, ry * 3141.592654f / 180.0, Z_SPEED_1_VISION);
                //抓子张开, 滑台上升到某个高度, 使摄像头能看到盒子
                txKylinMsg_ec_Fun(0, 0, 0, 0);
            }
            //fixed Ultrasonic
            //fixed 超声波引导
            if (finishDetectBoxFlag == true && finishFixedUltrasonicFlag == false)
            {
                coutLogicFlag = 2;
                detection_mode = 0;
                txKylinMsg.cbus.fs &= ~(1u << 30);
                
                //如果 fixed 超声波达不到盒子, 则小车向左移动, 如果能够达到盒子, 则向前移动
                // TODO: 测试 fixed 超声波测到的距离为多少时, 表明超声波无法打到盒子
                // TODO: 测试小车车身倾斜的情况下, 本判定条件是否会生效
                if (sr04maf[SR04_IDX_F].avg > 900)
                {
                    txKylinMsg_xyz_Fun(-200, 200, 0, 0, 0, 0);
                }
                else
                {
                    txKylinMsg_xyz_Fun(0, 0, sr04maf[SR04_IDX_F].avg, FIXED_ULTRASONIC_MOVE_SPEED, 0, 0);
                }
                //抓子和滑台位置保持不变
                txKylinMsg_ec_Fun(0, 0, 0, 0);
            }
            //left right Ultrasonic
            //左右超声波对准
            if (finishFixedUltrasonicFlag == true && finish_LR_UltrasonicFlag == false)
            {
                //finishDetectBoxFlag = false;
                coutLogicFlag = 3;
                detection_mode = 0;

                txKylinMsg.cbus.fs &= ~(1u << 30);
                txKylinMsg_xyz_Fun(moveDistance, LRSPEED, 0, 0, 0, 0);

                //滑台下降到最低点, 便于超声波进行对准
                txKylinMsg_ec_Fun(GraspBw - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, GraspOp, 0);
            }
            //squares detection finished, begin detect green area
            // 质心检测前的滑台位置调整
            if (finish_LR_UltrasonicFlag == true && finishGraspBeforeCentroidFlag == false)
            {
                //finishFixedUltrasonicFlag = false;
                finishGraspBeforeCentroidFlag = true;
                detection_mode = 0;
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

                txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);

                //TODO: 开启这部分之后, 需要修改滑台上升位置
                txKylinMsg_ec_Fun(GraspBw - 80 - kylinMsg.cbus.gp.e, 0, 0, 0);
            }
            //质心检测
            if (finishGraspBeforeCentroidFlag == true && finishDetectCentroidFlag == false)
            {
                //finishFixedUltrasonicFlag = false;
                finishDetectCentroidFlag = true;
                coutLogicFlag = 4;
                detection_mode = 2;                //打开视觉,检测质心
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

                //左右移动
                txKylinMsg_xyz_Fun(tx, 100, 0, 0, 0, 0);

                //注意抓子位置
                txKylinMsg_ec_Fun(GraspBw - DETECT_SQUARE_GRASP_POSITION - kylinMsg.cbus.gp.e, 0, 0, 0);
            }
            //image detection finished. only go forward
            //mobile 超声波引导小车抓盒子
            if (finishDetectCentroidFlag == true && finishMobleUltrasonicFlag == false)
            {
                calibPx();
                calibPz90();
                coutLogicFlag = 5;
                //finish_LR_UltrasonicFlag = false;
                detection_mode = 0;                //关闭视觉
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
                
                //mobil 超声波引导小车前进
                txKylinMsg_xyz_Fun(0, 0, sr04maf[SR04_IDX_M].avg, MOBILE_ULTRASONIC_MOVE_SPEED, 0, 0);

                //抓子放到在最低点
                txKylinMsg_ec_Fun(GraspBw - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, 0, 0);
            }
            //begin grasp
            //开始抓盒子
            if (finishMobleUltrasonicFlag == true && finishGraspFlag == false)
            {
                coutLogicFlag = 6;
                //finishDetectCentroidFlag = false;   //clear the flag
                detection_mode = 0; //关闭视觉
                txKylinMsg.cbus.fs &= ~(1u << 30);
                
                //小车不动
                txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);

                //开始抓盒子, 抓子合拢
                txKylinMsg_ec_Fun(0, 0, GraspCl, GRASP_CLOSE_SPEED);
            }
            //finish grasp. begin pull up.
            //滑台上升
            if (finishGraspFlag == true && finishSlidFlag == false)
            {
                coutLogicFlag = 7;
                //finishMobleUltrasonicFlag = false;
                detection_mode = 0; //关闭视觉
                txKylinMsg.cbus.fs &= ~(1u << 30);

                txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);
                //TODO: 滑台上升位置
                txKylinMsg_ec_Fun((GraspBw + GraspTp) / 2.0 - kylinMsg.cbus.gp.e, GRASP_UP_SPEED_HAVE_BOX, 0, 0);
            }
            //抓到盒子并抬高了滑台, 进入下一届阶段，回到原点
            if (finishSlidFlag == true)
            {
                txKylinMsg.cbus.gv.e = 0;
                lastWs = workState;
                rstRmp();
                workState = 2; //切换到下一阶段
                finishAbsoluteMoveFlag = false;
            }

            break;
        case 2:
            //if (lastWs != 2) rstRmp();
            //小车回到原点, 车头朝向前方
            ramp = genRmp();
            workState2_Num++;
            coutLogicFlag = 8;
            detection_mode = 0;             //关闭视觉
            txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式

            if (absoluteDistance < 10)
            {
                txKylinMsg_xyz_Fun(kylinOdomCalib.cbus.cp.x, X_SPEED_2 * ramp, kylinOdomCalib.cbus.cp.y, Y_SPEED_2 * ramp, 0 + kylinOdomCalib.cbus.cp.z, Z_SPEED_2 * ramp);
            }
            else
            {
                txKylinMsg_xyz_Fun(kylinOdomCalib.cbus.cp.x, X_SPEED_2 * ramp, kylinOdomCalib.cbus.cp.y, Y_SPEED_2 * ramp, 0, 0);
            }
            //保持抓子不变
            txKylinMsg_ec_Fun((GraspBw + GraspTp) / 2.0, 0, GraspCl, 0);

            if (finishAbsoluteMoveFlag == true && workState2_Num != 1) //完成绝对位置控制模式
            {
                rstRmp();
                workState = 3; //切换到下一阶段
                //值覆盖
                txKylinMsg_xyz_Fun(100 + kylinOdomCalib.cbus.cp.x, 1, 1511 + kylinOdomCalib.cbus.cp.y, 1, 0, 0);
                finishAbsoluteMoveFlag == false;
                //finishAbsoluteMoveFlag_Put = false;
            }
            break;
        case 3:
            //小车从原点达到基地区第一堆盒子的位置, 第一堆盒子的坐标 (0,3485)
            ramp = genRmp();
            if (finishAbsoluteMoveFlag_Put == false)
            {
                workState3_Num++;
                coutLogicFlag = 9;
                if (boxNum == 1)
                {
                    detection_mode = 0;             //关闭视觉
                    txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式

                    //到达目的地(基地区位置)
                    //基地区坐标为(AXISX, AXISY)
                    txKylinMsg_xyz_Fun(AXISX + kylinOdomCalib.cbus.cp.x, X_SPEED_3 * ramp, AXISY + kylinOdomCalib.cbus.cp.y, Y_SPEED_3 * ramp, 0, 0);
                    txKylinMsg_ec_Fun(0, 0, 0, 0);
                }
                else
                {
                    videoMove_PutBox();
                }
            }
            //到达指定位置，放下盒子
            if (finishAbsoluteMoveFlag_Put == true && finishSlidBwFlag == false)
            {
                coutLogicFlag = 10;
                txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式

                txKylinMsg_xyz_Fun(0 + kylinOdomCalib.cbus.cp.x, 0, 1511, 0, 0, 0);
                //TODO: 不同模式使用不同的放盒子高度
                //堆叠模式选择:   1 -> 2+2+2+2=8, 2 -> 2+1+2+1+2=8, 3 -> 1+1+1+1+1+1+1+1=8
                if(PUTBOX_MODE == 1)
                {
                    if (boxNum != 4)
                    {
                        txKylinMsg_ec_Fun((GraspBw - 15), GRASP_DOWN_SPEED_HAVE_MANY_BOX, 0, 0);
                    }
                    else
                    {
                        txKylinMsg_ec_Fun((GraspBw - 15 - 400), GRASP_DOWN_SPEED_HAVE_MANY_BOX, 0, 0);
                    }
                }
                if(PUTBOX_MODE == 2)
                {
                    if(boxNum == 1 || boxNum == 3)
                    {
                        txKylinMsg_ec_Fun(GraspBw - 15, GRASP_DOWN_SPEED_HAVE_MANY_BOX, 0, 0);
                    }
                    else if(boxNum == 2 || boxNum == 4)
                    {
                        txKylinMsg_ec_Fun(GraspBw - 15 - 400, GRASP_DOWN_SPEED_HAVE_MANY_BOX, 0, 0);
                    }
                    else if(boxNum == 5)
                    {
                        txKylinMsg_ec_Fun(GraspBw - 15 - 600, GRASP_DOWN_SPEED_HAVE_MANY_BOX, 0, 0);
                    }

                }
                if(PUTBOX_MODE == 3)
                {
                    txKylinMsg_ec_Fun(GraspBw - 15 - 200 * ((boxNum - 1) % 3), GRASP_DOWN_SPEED_HAVE_MANY_BOX, 0, 0);
                }
            }
            //松开抓子
            if (finishSlidBwFlag == true && finishGraspOpFlag == false)
            {
                calibPx();
                calibPz();
                coutLogicFlag = 11;
                txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
                txKylinMsg_xyz_Fun(0 + kylinOdomCalib.cbus.cp.x, 0, 1511, 0, 0 + kylinOdomCalib.cbus.cp.z, 0);
                txKylinMsg_ec_Fun(0, 0, GraspOp, GRASP_OPEN_SPEED);
            }
            if (finishGraspOpFlag == true)
            {
                if (boxNum == 8)
                {
                    coutLogicFlag = INT_MAX - 1;
                    videoMove_PutBox2toBox1();
                }
                if (finish_PutBox2toBox1_3to2 == true)
                {
                    coutLogicFlag = INT_MAX;
                    putBoxNum = 2;
                    videoMove_PutBox2toBox1();
                }
                if (finish_PutBox2toBox1_2to1 == true || boxNum != 8)
                {
                    lastWs = workState;
                    rstRmp();
                    workState = 4; //进入下一阶段
                    txKylinMsg_xyz_Fun(0 + kylinOdomCalib.cbus.cp.x, 10, 0 + kylinOdomCalib.cbus.cp.y, 10, 0, 0);
                    txKylinMsg_ec_Fun(GraspTp, 10, 0, 0);
                }
            }
            break;
        case 4:
            //先后退
            if (finishBackMoveFlag == false)
            {
                detection_mode = 0;
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
                //直接后退到某个位置
                txKylinMsg_xyz_Fun(0, 0, -200, DIRECT_BACK_MOVE_SPEED, 0, 0);
                //因为抓子当前处于最低点, 为了能够时候 fixed 超声波, 现将抓子抬高
                if (firstBoxJudgeFun())
                {
                    txKylinMsg_ec_Fun(GraspBw - DIRECT_BACK_MOVE_GRASP_UP_POSITION - kylinMsg.cbus.gp.e, GRASP_UP_SPEED, GraspOp, GRASP_OPEN_SPEED);
                }
                else
                {
                    txKylinMsg_ec_Fun(0, 0, GraspOp, GRASP_OPEN_SPEED);
                }
            }

            if (finishBackMoveFlag == true && finishAbsoluteMoveFlag_BackOrigin == false)
            {
                ramp = genRmp();
                workState4_Num++;
                // lineflag++;
                detection_mode = 0; //关闭视觉
                coutLogicFlag = 12;
                txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
                // if (lineflag < 500) //4.6
                //     txKylinMsg.cbus.cv.x = 0;
                // else
                // {
                //     txKylinMsg.cbus.cv.x = ;
                // }
                // 回原点
                txKylinMsg_xyz_Fun(0 + kylinOdomCalib.cbus.cp.x, X_SPEED_4 * ramp, 0 + kylinOdomCalib.cbus.cp.y, Y_SPEED_4 * ramp, 0 + kylinOdomCalib.cbus.cp.z, ZSPEED);
                // 滑台升高位置
                txKylinMsg_ec_Fun(GraspBw - DETECT_SQUARE_GRASP_POSITION - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, GraspOp, 0);
            }

            if (finishAbsoluteMoveFlag_BackOrigin == true)
            {
                lastWs = workState;
                rstRmp();
                workState = 0; //进入下一阶段
                lineflag = 0;
                finishAbsoluteMoveFlag = false; //完成本阶段的绝对位置控制
                finishDetectBoxFlag = false;
                finishDetectCentroidFlag = false;  //完成质心检测
                finishMobleUltrasonicFlag = false; //超声波到达极限距离
                finishGraspFlag = false;           //抓子是否合拢
                finishSlidFlag = false;            //滑台是否达到指定高度
                finishGraspOpFlag = false;         //抓子是否合拢
                finishSlidBwFlag = false;          //滑台是否达到指定高度
                finishAbsoluteMoveFlag_Put = false;
                finishFixedUltrasonicFlag = false;
                finishGraspBwFlag_PutBox = false;
                finish_LR_UltrasonicFlag = false;
                finish_L_UltrasonicFlag = false;
                finish_R_UltrasonicFlag = false;
                finishAbsoluteMoveFlag_BackOrigin = false;
                finishDetectBoxFlag_PutBox = false;
                finishFixedUltrasonicFlag_1_PutBox = false;
                finish_LR_UltrasonicFlag_PutBox = false;
                finishFixedUltrasonicFlag_2_PutBox = false;
                finishSlidTpFlag_PutBox = false;
                finishBackMoveFlag = false;
                finishGraspBeforeCentroidFlag = false;
                workState4_Num = 0, workState3_Num = 0, workState2_Num = 0, workState1_Num = 0, workState0_Num = 0;
                boxNum++;
            }
            break;
        default:
            break;
        }
        //根据最大搬运盒子数量, 修改这里的值
        if (boxNum == MAX_BOXNUM + 1)
        {
            missionEndTimeUs = currentTimeUs();
            cout << "mission time cost:" << (missionEndTimeUs - missionStartTimeUs) * 1e-6 << endl;
            break;
        }
        //int c = waitKey(1);

        //if ((char)c == 'q')
        //break;
    }

    //if(capture.isOpened())
    //	capture.release();
    capture.closeStream();
    disconnect_serial();
    cout << "function done!" << endl;

    return 0;
}

/*************************************************************************
*  函数名称：videoMove_PutBox
*  功能说明：单次盒子放置函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017-05-17
*************************************************************************/
void videoMove_PutBox()
{
    //矩形检测
    if (finishDetectBoxFlag_PutBox == false)
    {
        ramp = genRmp();
        coutLogicFlag_PutBox = 9.1;
        detection_mode = 1; //打开视觉,检测矩形
        //cout<<"detection_mode"<<(int)detection_mode<<endl;
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
        //矩形检测引导盒子
        txKylinMsg_xyz_Fun(tx - DIFFCONST, X_SPEED_3 * ramp, tz, Y_SPEED_3 * ramp, ry * 3141.592654f / 180, Z_SPEED_3_VISION);
        txKylinMsg_ec_Fun((GraspBw + GraspTp) / 2.0 - kylinMsg.cbus.gp.e, 0, GraspCl, 0);
    }
    //fixed Ultrasonic
    //fixed 超声波引导检测
    if (finishDetectBoxFlag_PutBox == true && finishFixedUltrasonicFlag_1_PutBox == false)
    {
        coutLogicFlag_PutBox = 9.2;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        
        //如果 fixed 超声波没有打到盒子, 小车向左移动
        if (sr04maf[SR04_IDX_F].avg > 650)
        {
            txKylinMsg_xyz_Fun(-200, 150, sr04maf[SR04_IDX_F].avg, 0, 0, 0);
        }
        else
        {
            txKylinMsg_xyz_Fun(0, 0, sr04maf[SR04_IDX_F].avg, FIXED_ULTRASONIC_MOVE_SPEED, 0, 0);
        }
        //抓子不动
        txKylinMsg_ec_Fun((GraspBw + GraspTp) / 2.0 - kylinMsg.cbus.gp.e, 0, GraspCl, 0);
    }
    //add Grasp Flag
    //抓子下降到最低点
    if (finishFixedUltrasonicFlag_1_PutBox == true && finishGraspBwFlag_PutBox == false)
    {
        coutLogicFlag_PutBox = 9.21;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);
        txKylinMsg_ec_Fun(GraspBw - 30 - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, GraspCl, 0);
    }
    //左右超声波对准盒子
    if (finishGraspBwFlag_PutBox == true && finish_LR_UltrasonicFlag_PutBox == false)
    {
        coutLogicFlag_PutBox = 9.3;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg_xyz_Fun(moveDistance, LRSPEED, 0, 0, 0, 0);
        txKylinMsg_ec_Fun(GraspBw - 30 - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, 0, 0);
    }
    //非每一堆的第一个盒子, 进入此 if 语句
    if (unFirstBoxJudgeFun())
    {
        if (finish_LR_UltrasonicFlag_PutBox == true && finishSlidTpFlag_PutBox == false)
        {
            coutLogicFlag_PutBox = 9.4;
            detection_mode = 0;                //关闭视觉
            txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

            txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);
            if (PUTBOX_MODE == 1)
            {
                if (boxNum != 4)
                {
                        txKylinMsg_ec_Fun((GraspBw - 15 - 20) - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED_HAVE_BOX, 0, 0);
                }
                else
                {
                        txKylinMsg_ec_Fun((GraspBw - 15 - 20 - 400) - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED_HAVE_BOX, 0, 0);
                }
            }
            if (PUTBOX_MODE == 2)
            {
                if (boxNum == 1 || boxNum == 3)
                {
                    txKylinMsg_ec_Fun(GraspBw - 15 - 20 - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED_HAVE_BOX, 0, 0);
                }
                else if (boxNum == 2 || boxNum == 4)
                {
                    txKylinMsg_ec_Fun(GraspBw - 15 - 20 - 400 - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED_HAVE_BOX, 0, 0);
                }
                else if (boxNum == 5)
                {
                    txKylinMsg_ec_Fun(GraspBw - 15 - 20 - 600 - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED_HAVE_BOX, 0, 0);
                }
            }
            if (PUTBOX_MODE == 3)
            {
                txKylinMsg_ec_Fun(GraspBw - 15 - 20 - 200 * ((boxNum - 1) % 3) - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED_HAVE_BOX, 0, 0);
            }
        }
        if (finishSlidTpFlag_PutBox == true && finishFixedUltrasonicFlag_2_PutBox == false)
        {
            coutLogicFlag_PutBox = 9.5;
            detection_mode = 0;                //关闭视觉
            txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
            if (sr04maf[SR04_IDX_F].avg > 650)
            {
                txKylinMsg_xyz_Fun(-200, 150, sr04maf[SR04_IDX_F].avg, 0, 0, 0);
            }
            else
            {
                txKylinMsg_xyz_Fun(0, 0, sr04maf[SR04_IDX_F].avg, FIXED_ULTRASONIC_MOVE_SPEED, 0, 0);
            }
            txKylinMsg_ec_Fun(0, 0, 0, 0);
        }
        if (finishFixedUltrasonicFlag_2_PutBox == true)
        {
            coutLogicFlag_PutBox = 9.6;
            finishAbsoluteMoveFlag_Put = true;
        }
    }
    if (firstBoxJudgeFun() && finish_LR_UltrasonicFlag_PutBox == true)
    {
        if (finishGraspBwFlag_PutBox == true)
        {
            finishAbsoluteMoveFlag_Put = true;
        }
    }
}

/*************************************************************************
*  函数名称：videoMove_PutBox2toBox1
*  功能说明：盒子堆叠函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017-05-17
*************************************************************************/
void videoMove_PutBox2toBox1()
{
    //后退到550
    if (finishBackMoveFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.1;
        detection_mode = 0;
        //cout<<"detection_mode"<<(int)detection_mode<<endl;
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

        //后退距离和速度
        txKylinMsg_xyz_Fun(0, 0, -200, DIRECT_BACK_MOVE_SPEED, 0, 0);
        //打开抓子
        txKylinMsg_ec_Fun(0, 0, GraspOp, GRASP_OPEN_SPEED);
    }
    //降下抓子
    if (finishBackMoveFlag_PutBox2toBox1 == true && finishGraspBwFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.2;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);

        txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);

        //抓子下降到最低点
        txKylinMsg_ec_Fun(GraspBw - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, 0, 0);
    }
    //左右超声波对准
    if (finishGraspBwFlag_PutBox2toBox1 == true && finish_LR_UltrasonicFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.3;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);

        txKylinMsg_xyz_Fun(moveDistance, LRSPEED, 0, 0, 0, 0);

        txKylinMsg_ec_Fun(GraspBw - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, 0, 0);
    }
    //质心对准
    //squares detection finished, begin detect green area
    if (finish_LR_UltrasonicFlag_PutBox2toBox1 == true && finishDetectCentroidFlag_PutBox2toBox1 == false)
    {
        finishDetectCentroidFlag_PutBox2toBox1 = true; //close DetectCentroid
        //finishFixedUltrasonicFlag = false;
        coutLogicFlag_PutBox2toBox1 = 10.4;
        detection_mode = 0;                //打开视觉,检测质心
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
        txKylinMsg_xyz_Fun(tx, 100, 0, 0, 0, 0);
        txKylinMsg_ec_Fun(GraspBw - DETECT_SQUARE_GRASP_POSITION - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, GraspOp, 0);
    }
    //moble超声波引导
    if (finishDetectCentroidFlag_PutBox2toBox1 == true && finishMobleUltrasonicFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.5;
        detection_mode = 0;                //关闭视觉
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
        txKylinMsg_xyz_Fun(0, 0, sr04maf[SR04_IDX_M].avg, MOBILE_ULTRASONIC_MOVE_SPEED, 0, 0);
        txKylinMsg_ec_Fun(GraspBw - kylinMsg.cbus.gp.e, GRASP_DOWN_SPEED, 0, 0);
    }
    //合拢抓子
    //begin grasp.
    if (finishMobleUltrasonicFlag_PutBox2toBox1 == true && finishGraspFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.6;
        //finishDetectCentroidFlag = false;   //clear the flag
        detection_mode = 0; //关闭视觉
        txKylinMsg.cbus.fs &= ~(1u << 30);

        txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);
        txKylinMsg_ec_Fun(0, 0, GraspCl, GRASP_CLOSE_SPEED);
    }
    //升高滑台
    //finish grasp. begin pull up.
    if (finishGraspFlag_PutBox2toBox1 == true && finishSlidFlag_PutBox2toBox1 == false)
    {
        //while(numDelay--);
        coutLogicFlag_PutBox2toBox1 = 10.7;
        //finishMobleUltrasonicFlag = false;
        detection_mode = 0; //关闭视觉
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);
        txKylinMsg_ec_Fun(GraspTp - kylinMsg.cbus.gp.e, GRASP_UP_SPEED_HAVE_BOX, GraspCl, 0);
    }
    if (finishSlidFlag_PutBox2toBox1 == true && finishFixedUltrasonicFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.8;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        if (sr04maf[SR04_IDX_F].avg > 400)
        {
            txKylinMsg_xyz_Fun(-200, 150, sr04maf[SR04_IDX_F].avg, 0, 0, 0);
        }
        else
        {
            txKylinMsg_xyz_Fun(0, 0, sr04maf[SR04_IDX_F].avg, FIXED_ULTRASONIC_MOVE_SPEED, 0, 0);
        }
        txKylinMsg_ec_Fun(0, 0, 0, 0);
    }
    if (finishFixedUltrasonicFlag_PutBox2toBox1 == true && finishGraspOpFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.9;
        //finishDetectCentroidFlag = false;   //clear the flag
        detection_mode = 0; //关闭视觉
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg_xyz_Fun(0, 0, 0, 0, 0, 0);
        txKylinMsg_ec_Fun(GraspBw - kylinMsg.cbus.gp.e, 0, GraspOp - kylinMsg.cbus.gp.c, GRASP_OPEN_SPEED);
    }
    if (finishGraspOpFlag_PutBox2toBox1 == true && putBoxNum == 1)
    {
        finish_PutBox2toBox1_3to2 = true;
        coutLogicFlag = INT_MAX;
        finishBackMoveFlag_PutBox2toBox1 = false;
        finishGraspBwFlag_PutBox2toBox1 = false;
        finish_LR_UltrasonicFlag_PutBox2toBox1 = false;
        finishDetectCentroidFlag_PutBox2toBox1 = false;
        finishMobleUltrasonicFlag_PutBox2toBox1 = false;
        finishGraspFlag_PutBox2toBox1 = false;
        finishSlidFlag_PutBox2toBox1 = false;
        finishFixedUltrasonicFlag_PutBox2toBox1 = false;
        finishGraspOpFlag_PutBox2toBox1 = false;
    }
    if (finishGraspOpFlag_PutBox2toBox1 == true && putBoxNum == 2)
    {
        finish_PutBox2toBox1_2to1 = true;
    }
}
