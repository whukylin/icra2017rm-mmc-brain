#include "main.h"
#include <math.h>
#include "RMVideoCapture.hpp"
#include "Markdetection.h"
#include "CMT.h"
#include "gui.h"
#include <string>
#include "kylindef.h"

// 滑台上升速度和下降速度(测试阶段，降低下降速度, 全局速度)
#define GRASP_UP_SPEED 1200
#define GRASP_DOWN_SPEED 800

// 抓子夹有盒子时, 滑台上升和下降速度(普遍降低)
#define GRASP_UP_SPEED_HAVE_BOX 600
#define GRASP_DOWN_SPEED_HAVE_BOX 600
#define GRASP_DOWN_SPEED_HAVE_MANY_BOX 400
#define GRASP_UP_SPEED_HAVE_MANY_BOX 400

// 全局滑台高度宏定义
#define DETECT_BOX_SLIDE_HEIGHT 0 //矩形检测时, 滑台高度
#define GRASP_BOX_SLIDE_HEIGHT 0  //抓取到盒子之后, 滑台高度
#define PUT_3_BOX_SLIDE_HEIGHT 0  //堆叠时, 第三个盒子高度位置的高度
#define HEAP_3_BOX_SLIDE_HEIGHT 0 //堆叠时, 第三个盒子高度位置的高度

//TODO: 检测矩形时, 滑台距离最低点的距离
#define DETECT_SQUARE_GRASP_POSITION 70

//TODO: 小车与盒子之间的距离小于多少时 从矩形检测引导小车切换到超声波引导

#define SQUARE_TO_FIXED_ULTRASONIC_DISTANCE 500

//TODO: 判定 fixed 超声波无法打到盒子并向左移动时的阈值
#define LEFT_MOVE_DISTANCE 0

// mobile 超声波引导时, 检测小车有无正确进入抓子的数组长度
#define DETECT_MOBILE_ERROR_LENGTH 2000
#define DETECT_MOBILE_ERROR_DISTANCE 10           //最长时间容忍的差值
#define DETECT_MOBILE_ERROR_MOVEBACK_DISTANCE 400 //出错后, 先后退, 后退的距离

//TODO: 抓住盒子之后, 滑台上升位置
#define GRASP_UP_HAVE_BOX_POSITION 210

// 进行矩形检测之前, 小车先直行一段距离, 降低小车矩形漏检率
#define AXISX_DIRECT 0
#define AXISY_DIRECT 1500

// 基地区坐标 axisX axisY
#define AXISX 0
#define AXISY 3200

//基地区新加盒子的坐标 addaxisX addaxisY
#define PY_MAN_CALIB_VAL 400
#define TWO_BOX_DIFF 300
#define ADDAXISX 0
#define ADDAXISY (AXISY - TWO_BOX_DIFF - PY_MAN_CALIB_VAL)

//TODO: 放置盒子的时候, 每一堆非第一个盒子放置的位置
#define FIXED_ULTRASONIC_2_PUTBOX 100
// 放置盒子的时候, 每一堆第一个盒子放置的位置
#define FIXED_ULTRASONIC_1_PUTBOX 500

// 放盒子时, 滑台偏移量(不让盒子挨着或者直接着地的偏移量)
#define SLIDE_DIFF 30
// 抓盒子时, 滑台身高位置
#define GRASP_BOX_POSITION 240

// 抓子松开和合拢速度
#define GRASP_OPEN_SPEED 10000
#define GRASP_CLOSE_SPEED 10000

//每一堆第一个盒子, 进行超声波对准之前, 抓子离地面的高度
#define PUT_FIRST_BOX_HEIGHT 150

// 放下盒子之后, 先后推, 后退的距离, 速度以及抓子抬高的高度
#define DIRECT_BACK_MOVE_DISTANCE 400
#define DIRECT_BACK_MOVE_SPEED 500
#define DIRECT_BACK_MOVE_GRASP_UP_POSITION 100

//
#define DIRECT_BACK_MOVE_DISTANCE_ADDBOX 1500

// 堆叠盒子时, 小车后退的距离
#define DIRECT_BACK_MOVE_DISTANCE_PUTBOX 400

// 堆叠盒子时, 放盒子的时候, fixed 超声波距离阈值
#define FIXED_ULTRASONIC_PUTBOX2TO1 100

// TODO:堆叠模式选择:   1 -> 2+2+2+2=8, 2 -> 2+1+2+1+2=8, 3 -> 1+1+1+1+1+1+1+1=8
#define PUTBOX_MODE 1

// 最大搬运盒子数量
// 盒子全部搬运过去之后, 进行堆叠之前, 总共有几堆盒子
#if PUTBOX_MODE == 1
#define MAX_BOXNUM 4
#define HEAP_NUM 3
#endif
#if PUTBOX_MODE == 2
#define MAX_BOXNUM 5
#define HEAP_NUM 2
#endif
#if PUTBOX_MODE == 3
#define MAX_BOXNUM 8
#define HEAP_NUM 3
#endif

// 抓盒子时, 角度补偿值
#define ANGLE_DIFF 0

//判断盒子是否完全进入抓子的模式: 1 -> 光电对管, 2-> 超声波, 3-> 融合
#define BOX_IN_GRASP_MODE 3
// 使用超声波判断盒子是否完全进入抓子时, 判断阈值
#define CLAW_CLOSE_SONAR_TRIGGER_DISTANCE 20

// 摄像头与小车轴心的固定偏移
#define DIFFCONST 101

// 小车旋转角度
#define ZROTATION90DEG 1572

// 小车运动速度宏定义(分阶段)
// 阶段 1 : 从原点出发, 抓盒子, 直到切换到 fixed 超声波
#define X_SPEED_1 400
#define Y_SPEED_1 800
#define Z_SPEED_1 1500
// 矩形检测引导小车旋转的速度
#define Z_SPEED_1_VISION 400
// fixed 超声波引导小车前进的速度
#define FIXED_ULTRASONIC_MOVE_SPEED 300
// left right 超声波对准盒子时, 相对位置控制左右移动的距离量以及左右移动的速度
#define LRDISTANCE 100 //100
#define LRSPEED 100    //200

//fixed 超声波打不到的时候, 小车向左移动的速度和距离
#define FIXED_DISTANCE 200
#define FIXED_SPEED 150

// mobile 超声波引导小车前进时, 小车移动速度
#define MOBILE_ULTRASONIC_MOVE_SPEED 300

// 阶段 2 : 小车抓取到盒子之后, 回原点的速度
#define X_SPEED_2 600
#define Y_SPEED_2 700
#define Z_SPEED_2 1400

// 阶段 3 : 小车拿着盒子, 到达基地区
#define X_SPEED_3 400
#define Y_SPEED_3 800
// First box is special, speed larger
#define Y_SPEED_3_FIRSTBOX 1000
#define Z_SPEED_3 1400
// 矩形检测引导小车旋转的速度
#define Z_SPEED_3_VISION 400

// 阶段 4 : 小车放下盒子, 回原点的速度
#define X_SPEED_4 600
#define Y_SPEED_4 1200
#define Z_SPEED_4 1400

#define YSPEED 1200 //forward speed
#define XSPEED 400
#define ZSPEED 1300

#define GRASPSPEED 1200

#define COM_PORT 1 //"COM7"
#define BUF_LEN 256
#define TIMEOUT 30
#define FRAME_N 20000
#define ADDSPEED 100

// 标志位 fs 宏定义
#define CONTROL_MODE_BIT 30 //相对位置和绝对位置控制模式对应的位数

using namespace cv;
using namespace std;
using cmt::CMT;
double missionStartTimeUs = 0;
double missionEndTimeUs = 0;

double absoluteDistance = 100;
double absuluteAngle = 100;
double absuluteGraspOpCl = 100;
double absuluteGrasp = 100;

int deltaAngle = 0;

double currentTimeUs()
{
    return cvGetTickCount() / cvGetTickFrequency();
}

float detectMobileError[DETECT_MOBILE_ERROR_LENGTH];
int currentMobileErrorCount = 0;
bool detectFlag = true;
int indexLoop = 0;
bool firstInCalibPy = false;
extern double ry, rz, rx;
extern double tx, ty, tz;
extern const char *wndname;
//Create a CMT object
CMT cmt0;
RMVideoCapture capture("/dev/video0", 3);
//VideoCapture capture;
int exp_time = 48;
int gain = 30;
int brightness_ = 10;
int whiteness_ = 86;
int saturation_ = 60;
//用于进行堆叠次数计数
int heapCount = HEAP_NUM - 1;
int8_t exit_flag = 0;
int workState = 0;
int grabBoxState;
int putBoxState;
int backwardState;
int graspOpClState;
int videoMovePutBoxState;
int UnFirstBox_PutBoxState;
int videoMove_PutBox2toBox1State;
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
volatile bool finishDetectBoxFlag = false;      //完成检测盒子(小车到了检测不到盒子的位置)
volatile bool finishDetectCentroidFlag = false; //完成质心检测
volatile bool finishDetectBoxFlag_PutBox = false;
volatile bool finishDetectCentroidFlag_PutBox2toBox1 = false;

volatile int GraspBwCout = 0;
volatile int GraspTpCout = 0;
volatile int absoluteDistanceCout = 0;
volatile bool finish_HeapBox = false;

string workStateCout;
string workStageCout;

int putBoxNum = 1;
int boxNum = 1, addboxNum = 0; //the num of the box
int GraspTp;
int GraspBw;
int GraspOp;
int GraspCl;
volatile double coutLogicFlag_PutBox = 0;
volatile double coutLogicFlag_PutBox2toBox1 = 0;
ZGyroMsg_t lastZGyroMsg;
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

        cout << "1. Ultrasonic: "
             << " Left: " << sr04maf[SR04_IDX_L].avg << "  Right: " << sr04maf[SR04_IDX_R].avg << "  Fixed: " << sr04maf[SR04_IDX_F].avg << "  Mobile: " << sr04maf[SR04_IDX_M].avg << endl;
        cout << "2. WorkState: "
             << "coutLogicFlag: " << coutLogicFlag << " coutLogicFlag_PutBox: " << coutLogicFlag_PutBox << " coutLogicFlag_PutBox2toBox1: " << coutLogicFlag_PutBox2toBox1 << endl;
        cout << "3. State: " << workStageCout << workStateCout << endl;
        cout << "abs(tz): " << abs(tz) << " lostFlag: " << lostFlag << endl;
        cout << "deltaAngle: " << deltaAngle << " zgyroMsg.angle: " << zgyroMsg.angle << " lastZGyroMsg.angle: " << lastZGyroMsg.angle << endl;
        cout << "------------------------------------------------------------------------------" << endl;
        // cout << "absoluteDistanceCout: " << absoluteDistanceCout << endl;
        // cout << "fflage: " << fflage << " tx:" << tx << " Vframe:" << CountVframe << endl;
        // cout << "Grasp_Ref: " << txKylinMsg.cbus.gp.c << "Grasp_Fdb: " << kylinMsg.cbus.gp.c << " SwitchFlag: " << kylinMsg.cbus.fs << " kylinMsg.cbus.fs & (1u << 2)): " << (kylinMsg.cbus.fs & (1u << 2)) << endl;
        // cout << "cl: " << GraspOp << " ch: " << GraspCl << endl;

        // detection_mode = 1; //for testing
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
            if ((sr04maf[SR04_IDX_M].avg < 410 && sr04maf[SR04_IDX_F].avg < 500) || (abs(tz) < 700 && (lostFlag == false) && CountVframe > 15))
            { //Usue ultra sonic distance for controlling. Detection_mode will be changed in main.
                finishDetectBoxFlag = true;
                CountVframe = 0;
            }
            else
            {
                finishDetectBoxFlag = false;
            }
            if (coutLogicFlag == 9 && ((sr04maf[SR04_IDX_M].avg < 410 && sr04maf[SR04_IDX_F].avg < 500) || (abs(tz) < 500 && (lostFlag == false) && CountVframe > 10)))
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

                //TODO: 更改为switch结构之后, 如果开启质心检测, 需要修改这一部分
                if (coutLogicFlag == INT_MAX)
                {
                    finishDetectCentroidFlag_PutBox2toBox1 = true;
                }
            }
            else
            {
                finishDetectCentroidFlag = false;
                finishDetectCentroidFlag_PutBox2toBox1 = false;
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
    //namedWindow("Tuning", 1);
    createTrackbar("exposure_time", wndname, &::exp_time, 100, on_expTracker);
    createTrackbar("gain", wndname, &::gain, 100, on_gainTracker);
    createTrackbar("whiteness", wndname, &::whiteness_, 100, on_whitenessTracker);
    createTrackbar("brightness_", wndname, &::brightness_, 100, on_brightnessTracker);
    createTrackbar("saturation", wndname, &::saturation_, 100, on_saturationTracker);
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
        imshow(wndname, img);
        int c = waitKey(20);
        if ((char)c == 'w')
            break;
    }
    //destroyWindow("Tuning");
    usleep(1000);
    return true;
    //RMVideoCapture cap("/dev/video0", 3);
}

void logicInit()
{
    txKylinMsg.cbus.fs &= ~(1u << CONTROL_MODE_BIT); //切换到相对位置控制模式
                                                     //detection_mode = 0;                //摄像头关闭
                                                     //cout << "Logic init finish!!" << endl;
}

#define ENABLE_SONAR(N)                              \
    do                                               \
    {                                                \
        Flag_Set(&txKylinMsg.cbus.fs, FS_SONAR_##N); \
    } while (0)

void enableSonars()
{
    ENABLE_SONAR(F);
    ENABLE_SONAR(M);
    ENABLE_SONAR(L);
    ENABLE_SONAR(R);
}

#define DISABLE_SONAR(N)                             \
    do                                               \
    {                                                \
        Flag_Clr(&txKylinMsg.cbus.fs, FS_SONAR_##N); \
    } while (0)

#define KEEP_SONAR_CTL_FLAG_BITS(N)                                                               \
    do                                                                                            \
    {                                                                                             \
        Flag_Det(&txKylinMsg.cbus.fs, FS_SONAR_##N, Flag_Get(&txKylinMsg.cbus.fs, FS_SONAR_##N)); \
    } while (0)

void keepSonarCtlFlagBits()
{
    KEEP_SONAR_CTL_FLAG_BITS(F);
    KEEP_SONAR_CTL_FLAG_BITS(M);
    KEEP_SONAR_CTL_FLAG_BITS(L);
    KEEP_SONAR_CTL_FLAG_BITS(R);
}

#define IS_SONAR_STATE_SYNCED(N) (Flag_Get(&kylinMsg.cbus.fs, FS_SONAR_##N) == Flag_Get(&txKylinMsg.cbus.fs, FS_SONAR_##N))

bool isSonarStateAllSynced()
{
    //return true;
    return IS_SONAR_STATE_SYNCED(F) && IS_SONAR_STATE_SYNCED(M) && IS_SONAR_STATE_SYNCED(L) && IS_SONAR_STATE_SYNCED(R);
}

void waitForSonarStateSynced()
{
    while (!isSonarStateAllSynced())
        ;
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
    enableSonars();
    waitForSonarStateSynced();
    return 1;
}

bool isPilingMissionDone = false;

void calibPyManually()
{
    kylinOdomCalib.cbus.cp.y += PY_MAN_CALIB_VAL;
}

void calibPyManuallyAgain()
{
    kylinOdomCalib.cbus.cp.y += TWO_BOX_DIFF;
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
*  函数名称：电子罗盘相关函数
*  功能说明：判断盒子是否完全进入抓子
*  参数说明：BOX_IN_GRASP_MODE: 1->光电对管, 2->超声波
*  函数返回：无
*  修改时间：2017-05-17
*************************************************************************/

void saveZGyroMsg()
{
    memcpy(&lastZGyroMsg, &zgyroMsg, sizeof(ZGyroMsg_t));
}

int getZGyroRelativeAngle()
{
    return (-zgyroMsg.angle + lastZGyroMsg.angle) * 10 * PI / 180;
}

int zgyroFusedYawPositionCtrl(int angle)
{
    deltaAngle = angle + getZGyroRelativeAngle();
    txKylinMsg.cbus.cp.z = kylinMsg.cbus.cp.z + deltaAngle;
    return deltaAngle;
}

int zgyroFusedYawPositionCtrlOnlyRet(int angle)
{
    deltaAngle = angle + getZGyroRelativeAngle();
    return deltaAngle;
}

/*************************************************************************
*  函数名称：switchFlagFun
*  功能说明：判断盒子是否完全进入抓子
*  参数说明：BOX_IN_GRASP_MODE: 1->光电对管, 2->超声波
*  函数返回：无
*  修改时间：2017-05-17
*************************************************************************/
bool switchFlagFun()
{
    if (BOX_IN_GRASP_MODE == 1) //使用光电对管
    {
        return (kylinMsg.cbus.fs & (1u << 2));
    }
    else if(BOX_IN_GRASP_MODE == 2) //使用 mobile 超声波
    {
        return (sr04maf[SR04_IDX_M].avg < CLAW_CLOSE_SONAR_TRIGGER_DISTANCE);
    }
    else
    {
        return ((sr04maf[SR04_IDX_M].avg < CLAW_CLOSE_SONAR_TRIGGER_DISTANCE) || (kylinMsg.cbus.fs & (1u << 2)));   
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
    txKylinMsg.cbus.cp.x = cpx; //x 左右移动
    txKylinMsg.cbus.cv.x = cvx;
    txKylinMsg.cbus.cp.y = cpy; //y 前后移动
    txKylinMsg.cbus.cv.y = cvy;
    txKylinMsg.cbus.cp.z = cpz; //转角
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
    txKylinMsg.cbus.gp.e = gpe; //滑台
    txKylinMsg.cbus.gv.e = gve;
    txKylinMsg.cbus.gp.c = gpc; //抓子
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
/*************************************************************************
*  函数名称：detectMobileError()
*  功能说明：检测mobile超声波引导时, 盒子有没有正确进入抓子的函数
*  参数说明：mobile mobile超声波测得的距离, 在使用这个函数之前, 要先对 noFirstIn 置 false
*  函数返回：有无正确进入抓子的标志位
*  修改时间：2017-05-24
*************************************************************************/
bool detectMobileErrorFun(float mobile)
{
    detectMobileError[currentMobileErrorCount] = mobile;
    currentMobileErrorCount++;
    if (currentMobileErrorCount == DETECT_MOBILE_ERROR_LENGTH)
    {
        if (abs(detectMobileError[0] - detectMobileError[DETECT_MOBILE_ERROR_LENGTH - 1]) > DETECT_MOBILE_ERROR_DISTANCE || mobile < 150)
        {
            return true;
        }
        else
        {
            return false;
        }
        currentMobileErrorCount = 0;
    }
}

/*************************************************************************
*  函数名称：enableSonars
*  功能说明：控制超声波开关的函数
*  参数说明：fixed, mobille, left, right, 1 开, 0 断
*  函数返回：无
*  修改时间：2017-05-21
*************************************************************************/
void enableSonarsFun(int fixed, int mobile, int left, int right)
{
    // fixed = mobile = left = right = 1;
    // if (fixed == 1)
    // {
    //     ENABLE_SONAR(F);
    // }
    // else
    // {
    //     DISABLE_SONAR(F);
    // }
    // if (mobile == 1)
    // {
    //     ENABLE_SONAR(M);
    // }
    // else
    // {
    //     DISABLE_SONAR(M);
    // }
    // if (left == 1)
    // {
    //     ENABLE_SONAR(L);
    // }
    // else
    // {
    //     DISABLE_SONAR(L);
    // }
    // if (right == 1)
    // {
    //     ENABLE_SONAR(R);
    // }
    // else
    // {
    //     DISABLE_SONAR(R);
    // }
}

bool isZGyroFusedPositionCtrlStart = false;

void rstYaw()
{
    isZGyroFusedPositionCtrlStart = false;
    saveZGyroMsg();
}
int main(int argc, char **argv)
{
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
        //return -1;
    }

    //updateOdomCalib();
   // logicInit(); //逻辑控制初始化
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

        capture >> frame;
        if (frame.empty())
            continue;

        int dif_x = 0, dif_y = 0;
        Mat src = frame.clone();
        findSquares(src, frame, squares);
        LocationMarkes(squares);
        drawSquares(frame, squares);
        int c = waitKey(2);
        if ((char)c == 'q')
            break;
    }
    capture.closeStream();
    return 0;
}

