#include "main.h"
#include <math.h>
#include "RMVideoCapture.hpp"
#include "Markdetection.h"
#define COM_PORT 1 //"COM7"
#define BUF_LEN 256
#define TIMEOUT 30
#define FRAME_N 20000
#define ADDSPEED 100

#define YSPEED 1000 //forward speed
#define XSPEED 500
#define ZSPEED 1000
#define GRASPSPEED 1000

using namespace cv;
using namespace std;

extern double ry, rz, rx;
extern double tx, ty, tz;
extern const char *wndname;

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
volatile bool finish_PutBox2toBox1 = false;
volatile bool finishBackMoveFlag = false;
volatile int GraspBwCout = 0;
volatile int GraspTpCout = 0;
volatile int absoluteDistanceCout = 0;
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
        Dnl_ProcSr04sMsg(&sr04sMsg);
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

#define RMP_CNT 200000
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
        //cout << "absoluteDistanceCout: " << absoluteDistanceCout << endl;
        //cout << "finishDetectBoxFlag_PutBox: " << finishDetectBoxFlag_PutBox << endl;
	cout << "coutLogicFlag: " << coutLogicFlag << " coutLogicFlag_PutBox: " << coutLogicFlag_PutBox << " coutLogicFlag_PutBox2toBox1: " << coutLogicFlag_PutBox2toBox1 << endl;
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
                    tx = 60;
                    ty = 0;
                    tz = 0;
                    rx = 0;
                    ry = 0;
                    rz = 0;
                }
            }
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
            Color_detect(src, dif_x, dif_y);
            tx = 2 * (dif_x - DIF_CEN);
            // txKylinMsg.cbus.cp.x = 10 * dif_x;
            // txKylinMsg.cbus.cp.y = 0;
            // txKylinMsg.cbus.cp.z = 0;
            if (abs(tx) < 20) //number of pixels
            {
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
void workStateFlagPrint()
{
    //cout << "workState: " << workState;
    //cout << " finishAbsoluteMoveFlag:" << finishAbsoluteMoveFlag << endl;
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
float Ultrasonic2Angle();
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
        return -1;
    }

    MyThread kylibotMsgPullerTread;
    MyThread kylibotMsgPusherTread;
    MyThread kylibotMarkDetectionTread;

    kylibotMsgPullerTread.create(KylinBotMsgPullerThreadFunc, NULL);
    kylibotMsgPusherTread.create(KylinBotMsgPusherThreadFunc, NULL);
    kylibotMarkDetectionTread.create(KylinBotMarkDetecThreadFunc, NULL);
    // txKylinMsg.cbus.cp.x = tx;     //小车左右
    // txKylinMsg.cbus.cv.x = 500;
    // txKylinMsg.cbus.cp.y = tz;     //小车前后
    // txKylinMsg.cbus.cv.y = 800;
    // txKylinMsg.cbus.cp.z = ry;     //小车旋转
    // txKylinMsg.cbus.cv.z = 500;
    // txKylinMsg.cbus.gp.e = ty;     //抓子高度
    // txKylinMsg.cbus.gv.e = 0;

    // KylinMsg.cbus.cp.x
    // kylinMsg.cbus.cp.y
    // kylinMsg.cbus.cp.z
    // kylinMsg.cbus.gp.e
    // kylinMsg.cbus.gp.c
    // kylinMsg.cbus.cv.x
    // kylinMsg.cbus.cv.y
    // kylinMsg.cbus.cv.z
    // kylinMsg.cbus.gv.e
    // kylinMsg.cbus.gv.c
    /* 经验值
     *        txKylinMsg.cbus.cp.x = tx;
     *        txKylinMsg.cbus.cv.x = 500;
     *        txKylinMsg.cbus.cp.y = tz;
     *        txKylinMsg.cbus.cv.y = 800;
     *        txKylinMsg.cbus.cp.z = ry * 3141.592654f / 180;
     *        txKylinMsg.cbus.cv.z = 500;
     *        txKylinMsg.cbus.gp.e = ty;
     *        txKylinMsg.cbus.gv.e = 0;
     */
    /*Flag变量汇总:
     *    finishAbsoluteMoveFlag      完成绝对位置移动
     *    finishDetectBoxFlag         完成检测盒子(小车到了检测不到盒子的位置)
     *    finishDetectCentroidFlag    完成质心检测
     *    finishMobleUltrasonicFlag   超声波到达极限距离
     *    finishGraspFlag             抓子是否合拢
     *    finishSlidFlag              滑台是否达到指定高度
     */
    updateOdomCalib();
    //cout << "w111s: " << workState << endl;
    logicInit(); //逻辑控制初始化
    //workStateFlagPrint(); //打印当前状态
    GraspTp = posCalibMsg.data.el;
    GraspBw = posCalibMsg.data.eh;
    GraspOp = posCalibMsg.data.cl;
    GraspCl = posCalibMsg.data.ch;

    GraspBwCout = GraspBw;
    GraspTpCout = GraspTp;
    //sr04maf[1].avg moble
    //sr04maf[0].avg fixed
    //sr04maf[2].avg left
    //sr04maf[3].avg right
    double absoluteDistance = 100;
    double absuluteAngle = 100;
    double absuluteGraspOpCl = 100;
    double absuluteGrasp = 100;
    int workState0_Num = 0, workState1_Num = 0, workState2_Num = 0, workState3_Num = 0, workState4_Num = 0;

    while ((!exit_flag)) //&&(capture.read(frame)))
    {
        //cout << "ws: " << workState << endl;
        updateOdomError();
        
        //如果当前处于绝对位置控制模式,进入判断条件
        if (txKylinMsg.cbus.fs & (1u << 30)) //0xFF == 1111 1111   0x80000000
        {

            absoluteDistance = pow(pow((kylinOdomError.cbus.cp.x), 2) + pow((kylinOdomError.cbus.cp.y), 2), 0.5);
            absoluteDistanceCout = absoluteDistance;
            absuluteAngle = abs(kylinOdomError.cbus.cp.z);
            absuluteGrasp = abs(kylinOdomError.cbus.gp.e);
            absuluteGraspOpCl = abs(kylinOdomError.cbus.gp.c);
            //cout<<"absoluteDistance" << absoluteDistance << endl;
            //cout<<"absuluteAngle" << absuluteAngle << endl;
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
            if (sr04maf[SR04_IDX_M].avg < 25)
            {
                finishMobleUltrasonicFlag = true;
            }
            //Grasp close
            if (kylinMsg.cbus.gp.c == GraspCl)
            {
                finishGraspFlag = true;
            }
            //Graps OpenfinishAbsoluteMoveFlag

            if (coutLogicFlag == 7 && kylinMsg.cbus.gp.e <= (GraspBw + GraspTp) / 2.0) //(GraspTp + GraspBw) / 2.f)
            //if(coutLogicFlag == 7 && abs(txKylinMsg.cbus.gp.e - kylinMsg.cbus.gp.e) < 20)
            {
                finishSlidFlag = true;
            }
            //Put box by video

            if (coutLogicFlag == 9 && sr04maf[SR04_IDX_F].avg < 500 && finishDetectBoxFlag_PutBox == true)
            {
                finishFixedUltrasonicFlag_1_PutBox = true;
            }
            if (coutLogicFlag == 9 && finishFixedUltrasonicFlag_1_PutBox == true && kylinMsg.cbus.gp.e >= GraspBw - 60)
            {
                finishGraspBwFlag_PutBox = true;
            }
            if (coutLogicFlag == 9 && sr04maf[SR04_IDX_L].avg > 300 && sr04maf[SR04_IDX_R].avg > 300 && finishGraspBwFlag_PutBox == true)
            {
                finish_LR_UltrasonicFlag_PutBox = true;
                moveDistance = 0;
            }
            if (coutLogicFlag == 9 && kylinMsg.cbus.gp.e <= GraspTp + 20 && finish_LR_UltrasonicFlag_PutBox == true)
            {
                finishSlidTpFlag_PutBox = true;
            }
            if (coutLogicFlag == 9 && sr04maf[SR04_IDX_F].avg < 130 && finishSlidTpFlag_PutBox == true)
            {
                finishFixedUltrasonicFlag_2_PutBox = true;
            }

            //fixed Ultra
            if (sr04maf[SR04_IDX_F].avg < 420 && finishDetectBoxFlag == true)
            {
                finishFixedUltrasonicFlag = true;
            }

            if (coutLogicFlag == 3 && sr04maf[SR04_IDX_L].avg > 300 && sr04maf[SR04_IDX_R].avg > 300)
            {
                finish_LR_UltrasonicFlag = true;
                moveDistance = 0;
            }
            if (sr04maf[SR04_IDX_L].avg > 300 && sr04maf[SR04_IDX_R].avg < 200)
            {
                moveDistance = 100;
            }
            if (sr04maf[SR04_IDX_L].avg < 300 && sr04maf[SR04_IDX_R].avg > 500)
            {
                moveDistance = -100;
            }
            if (workState == 4 && sr04maf[SR04_IDX_F].avg > 400)
            {
                finishBackMoveFlag = true;
            }
            //PutBox2toBox1
            if (coutLogicFlag == INT_MAX)
            {
                if (sr04maf[SR04_IDX_F].avg > 550 && coutLogicFlag_PutBox2toBox1 == 10.1)
                {
                    finishBackMoveFlag_PutBox2toBox1 = true;
                }
                if (kylinMsg.cbus.gp.e >= GraspBw - 10 && finishBackMoveFlag_PutBox2toBox1 == true)
                {
                    finishGraspBwFlag_PutBox2toBox1 = true;
                }
                if (sr04maf[SR04_IDX_L].avg > 300 && sr04maf[SR04_IDX_R].avg > 300 && finishGraspBwFlag_PutBox2toBox1 == true)
                {
                    finish_LR_UltrasonicFlag_PutBox2toBox1 = true;
                }
                if (sr04maf[SR04_IDX_M].avg < 25 && finish_LR_UltrasonicFlag_PutBox2toBox1 == true)
                {
                    finishMobleUltrasonicFlag_PutBox2toBox1 = true;
                }
                if (kylinMsg.cbus.gp.c == GraspCl && finishMobleUltrasonicFlag_PutBox2toBox1 == true)
                {
                    finishGraspFlag_PutBox2toBox1 = true;
                }
                if (kylinMsg.cbus.gp.e <= GraspTp + 5 && finishGraspFlag_PutBox2toBox1 == true)
                {
                    finishSlidFlag_PutBox2toBox1 = true;
                }
                if (sr04maf[SR04_IDX_F].avg < 130 && finishSlidFlag_PutBox2toBox1 == true)
                {
                    finishFixedUltrasonicFlag_PutBox2toBox1 = true;
                }
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
        while(true)
        {
            txKylinMsg.cbus.fs &= ~(1u << 30);
            txKylinMsg.cbus.cp.z = Ultrasonic2Angle() * 1000 - kylinMsg.cbus.cp.z; //1000 * PI / 2;// + kylinMsg.cbus.cp.z; //旋转90度
            txKylinMsg.cbus.cv.z = ZSPEED;
        }
        switch (workState)
        {
        case 0:
            coutLogicFlag = 0;
            //关闭视觉检测，小车在原点旋转90度，启动视觉检测（本阶段视觉关）仅仅只是//小车旋转
            detection_mode = 0;               //关闭视觉
            txKylinMsg.cbus.fs |= (1u << 30); //切换到绝对位置控制模式
            txKylinMsg.cbus.cp.x = 0 + kylinOdomCalib.cbus.cp.x;
            txKylinMsg.cbus.cv.x = 0;
            txKylinMsg.cbus.cp.y = 0 + kylinOdomCalib.cbus.cp.y;
            txKylinMsg.cbus.cv.y = 0;
            txKylinMsg.cbus.cp.z = -1572 + kylinOdomCalib.cbus.cp.z; //1000 * PI / 2;// + kylinMsg.cbus.cp.z; //旋转90度
            txKylinMsg.cbus.cv.z = ZSPEED * genRmp();
            txKylinMsg.cbus.gp.e = GraspBw - 70; //+ kylinOdomCalib.cbus.gp.e;
            txKylinMsg.cbus.gv.e = GRASPSPEED;
            txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
            txKylinMsg.cbus.gv.c = 8000;
            //absoluteDistance = pow(pow((kylinOdomError.cbus.cp.x), 2) + pow((kylinOdomError.cbus.cp.y), 2), 0.5);
            //absuluteAngle = abs(kylinOdomError.cbus.cp.z);
            if (absoluteDistance < 10 && absuluteAngle < 5.0f * PI / 2.0f) // && absuluteGrasp < 10)
            {
                //finishAbsoluteMoveFlag = true;
            }
            else
            {
                //finishAbsoluteMoveFlag = false;
            }
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
            //begin detect squares
            if (finishDetectBoxFlag == false)
            {
                ramp = genRmp();
                coutLogicFlag = 1;
                detection_mode = 1; //打开视觉,检测矩形
                //cout<<"detection_mode"<<(int)detection_mode<<endl;
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

                txKylinMsg.cbus.cp.x = tx - 60;
                txKylinMsg.cbus.cv.x = XSPEED * ramp;
                txKylinMsg.cbus.cp.y = tz;
                txKylinMsg.cbus.cv.y = YSPEED * ramp;
                ;
                txKylinMsg.cbus.cp.z = ry * 3141.592654f / 180;
                txKylinMsg.cbus.cv.z = ZSPEED;
                txKylinMsg.cbus.gp.e = 0;
                txKylinMsg.cbus.gv.e = 0;
                txKylinMsg.cbus.gp.c = 0; //抓子张开
                txKylinMsg.cbus.gv.c = 0;
            }
            //fixed Ultrasonic
            if (finishDetectBoxFlag == true && finishFixedUltrasonicFlag == false)
            {
                coutLogicFlag = 2;
                detection_mode = 0;
                txKylinMsg.cbus.fs &= ~(1u << 30);
                txKylinMsg.cbus.cp.x = 0;
                txKylinMsg.cbus.cv.x = 0;
                txKylinMsg.cbus.cp.y = sr04maf[0].avg; // - kylinMsg.cbus.cp.y;
                txKylinMsg.cbus.cv.y = 200;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
                txKylinMsg.cbus.gp.e = 0;
                txKylinMsg.cbus.gv.e = 0;
                txKylinMsg.cbus.gp.c = 0; //抓子张开
                txKylinMsg.cbus.gv.c = 0;
            }
            //left right Ultrasonic
            if (finishFixedUltrasonicFlag == true && finish_LR_UltrasonicFlag == false)
            {
                //finishDetectBoxFlag = false;
                coutLogicFlag = 3;
                detection_mode = 0;

                txKylinMsg.cbus.fs &= ~(1u << 30);
                txKylinMsg.cbus.cp.x = moveDistance;
                txKylinMsg.cbus.cv.x = 100;
                txKylinMsg.cbus.cp.y = 0;
                txKylinMsg.cbus.cv.y = 0;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
                txKylinMsg.cbus.gp.e = GraspBw - kylinMsg.cbus.gp.e;
                txKylinMsg.cbus.gv.e = GRASPSPEED;
                txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
                txKylinMsg.cbus.gv.c = 0;
            }
            //squares detection finished, begin detect green area
            if (finish_LR_UltrasonicFlag == true && finishDetectCentroidFlag == false)
            {
                //finishFixedUltrasonicFlag = false;
                coutLogicFlag = 4;
                detection_mode = 2;                //打开视觉,检测质心
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
                txKylinMsg.cbus.cp.x = tx;
                txKylinMsg.cbus.cv.x = 100;
                txKylinMsg.cbus.cp.y = 0;
                txKylinMsg.cbus.cv.y = 0;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
                txKylinMsg.cbus.gp.e = GraspBw - 70 - kylinMsg.cbus.gp.e;
                txKylinMsg.cbus.gv.e = GRASPSPEED;
                txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
                txKylinMsg.cbus.gv.c = 0;
            }
            //image detection finished. only go forward
            if (finishDetectCentroidFlag == true && finishMobleUltrasonicFlag == false)
            {
                calibPx();
                coutLogicFlag = 5;
                //finish_LR_UltrasonicFlag = false;
                detection_mode = 0;                //关闭视觉
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
                txKylinMsg.cbus.cp.x = 0;
                txKylinMsg.cbus.cv.x = 0;
                txKylinMsg.cbus.cp.y = sr04maf[SR04_IDX_M].avg;
                txKylinMsg.cbus.cv.y = 600;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
                txKylinMsg.cbus.gp.e = GraspBw - kylinMsg.cbus.gp.e;
                txKylinMsg.cbus.gv.e = GRASPSPEED;
                txKylinMsg.cbus.gp.c = 0; //抓子张开
                txKylinMsg.cbus.gv.c = 0;
            }
            //begin grasp.
            if (finishMobleUltrasonicFlag == true && finishGraspFlag == false)
            {
                coutLogicFlag = 6;
                //finishDetectCentroidFlag = false;   //clear the flag
                detection_mode = 0; //关闭视觉
                txKylinMsg.cbus.fs &= ~(1u << 30);
                txKylinMsg.cbus.cp.x = 0;
                txKylinMsg.cbus.cv.x = 0;
                txKylinMsg.cbus.cp.y = 0;
                txKylinMsg.cbus.cv.y = 0;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
                txKylinMsg.cbus.gp.e = 0; //(posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f - kylinMsg.cbus.gp.e;
                txKylinMsg.cbus.gv.e = 0;
                txKylinMsg.cbus.gp.c = GraspCl; //抓子合拢
                txKylinMsg.cbus.gv.c = 8000;
            }
            //finish grasp. begin pull up.
            if (finishGraspFlag == true && finishSlidFlag == false)
            {
                coutLogicFlag = 7;
                //finishMobleUltrasonicFlag = false;
                detection_mode = 0; //关闭视觉
                txKylinMsg.cbus.fs &= ~(1u << 30);
                txKylinMsg.cbus.cp.x = 0;
                txKylinMsg.cbus.cv.x = 0;
                txKylinMsg.cbus.cp.y = 0;
                txKylinMsg.cbus.cv.y = 0;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
                txKylinMsg.cbus.gp.e = (GraspBw + GraspTp)/2.0 - kylinMsg.cbus.gp.e; //(posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f; // - kylinMsg.cbus.gp.e; //滑台升高到 30cm (相对位置控制模式，要做一个反馈)  //10: unstable control
                txKylinMsg.cbus.gv.e = GRASPSPEED;
                txKylinMsg.cbus.gp.c = 0; //抓子合拢
                txKylinMsg.cbus.gv.c = 0;
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
            txKylinMsg.cbus.cp.x = kylinOdomCalib.cbus.cp.x;
            txKylinMsg.cbus.cv.x = XSPEED * ramp;
            txKylinMsg.cbus.cp.y = kylinOdomCalib.cbus.cp.y;
            txKylinMsg.cbus.cv.y = YSPEED * ramp;
            if (absoluteDistance < 10)
            {
                txKylinMsg.cbus.cp.z = 0 + kylinOdomCalib.cbus.cp.z; //1000 * PI / 2;// + kylinMsg.cbus.cp.z; //旋转90度
                txKylinMsg.cbus.cv.z = ZSPEED * ramp;
            }
            else
            {
                txKylinMsg.cbus.cv.z = 0;
            }

            //txKylinMsg.cbus.gp.e = (GraspBw + GraspTp)/2.0;
            //txKylinMsg.cbus.gv.e = 0;
            //txKylinMsg.cbus.gp.c = GraspCl;
            //txKylinMsg.cbus.gv.c = 0;
            if (finishAbsoluteMoveFlag == true && workState2_Num != 1) //完成绝对位置控制模式
            {
                rstRmp();
                workState = 3; //切换到下一阶段
                txKylinMsg.cbus.cp.x = 100 + kylinOdomCalib.cbus.cp.x;
                txKylinMsg.cbus.cv.x = 1;
                txKylinMsg.cbus.cp.y = 1511 + kylinOdomCalib.cbus.cp.y;
                txKylinMsg.cbus.cv.y = 1;
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
                    txKylinMsg.cbus.cp.x = 0 + kylinOdomCalib.cbus.cp.x;
                    txKylinMsg.cbus.cv.x = XSPEED * ramp;
                    txKylinMsg.cbus.cp.y = 2000 + kylinOdomCalib.cbus.cp.y; //3485 + kylinOdomCalib.cbus.cp.y;//3485 + kylinOdomCalib.cbus.cp.y;
                    txKylinMsg.cbus.cv.y = YSPEED * ramp;
                    txKylinMsg.cbus.cp.z = 0;
                    txKylinMsg.cbus.cv.z = 0;
                    txKylinMsg.cbus.gp.e = 0;
                    txKylinMsg.cbus.gv.e = 0;
                }
                if (boxNum == 2)
                {
                    videoMove_PutBox();
                }
                if (boxNum == 3)
                {
                    videoMove_PutBox();
                }
                if (boxNum == 4)
                {
                    videoMove_PutBox();
                }
                if (boxNum == 5)
                {
                    videoMove_PutBox();
                }
                if (boxNum == 6)
                {
                    videoMove_PutBox();
                }
            }

            //txKylinMsg.cbus.cp.y = 3485 + kylinOdomCalib.cbus.cp.y;//3485 + kylinOdomCalib.cbus.cp.y;

            //txKylinMsg.cbus.gp.e = (GraspBw + GraspTp)/2.0;
            //txKylinMsg.cbus.gv.e = 0;
            ////txKylinMsg.cbus.gp.c = GraspCl;
            //	txKylinMsg.cbus.gv.c = 0;

            // && workState3_Num >= 2000) //完成绝对位置控制模式

            //到达指定位置，放下盒子
            if (finishAbsoluteMoveFlag_Put == true && finishSlidBwFlag == false)
            {
                coutLogicFlag = 10;
                txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
                txKylinMsg.cbus.cp.x = 0 + kylinOdomCalib.cbus.cp.x;
                txKylinMsg.cbus.cv.x = 0;
                txKylinMsg.cbus.cp.y = 1511; //3485 + kylinOdomCalib.cbus.cp.y;
                txKylinMsg.cbus.cv.y = 0;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
                if (boxNum == 1)
                {
                    txKylinMsg.cbus.gp.e = GraspBw - 30;
                }
                if (boxNum == 2)
                {
                    txKylinMsg.cbus.gp.e = GraspBw - 30 - 210;
                }
                if (boxNum == 3)
                {
                    txKylinMsg.cbus.gp.e = GraspBw - 30 - 420;
                }
                if (boxNum == 4)
                {
                    txKylinMsg.cbus.gp.e = GraspBw - 30;
                }
                if (boxNum == 5)
                {
                    txKylinMsg.cbus.gp.e = GraspBw - 30 - 210;
                }
                if (boxNum == 6)
                {
                    txKylinMsg.cbus.gp.e = GraspBw - 30 - 420;
                }
                //txKylinMsg.cbus.gp.e = GraspBw - 40;
                txKylinMsg.cbus.gv.e = 400;
                //txKylinMsg.cbus.gp.c = GraspCl;
                //txKylinMsg.cbus.gv.c = 0;
            }
            //松开抓子
            if (finishSlidBwFlag == true && finishGraspOpFlag == false)
            {
                calibPx();
                coutLogicFlag = 11;
                txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
                txKylinMsg.cbus.cp.x = 0 + kylinOdomCalib.cbus.cp.x;
                txKylinMsg.cbus.cv.x = 0;
                txKylinMsg.cbus.cp.y = 1511; //3485 + kylinOdomCalib.cbus.cp.y;
                txKylinMsg.cbus.cv.y = 0;
                txKylinMsg.cbus.cp.z = 0 + kylinOdomCalib.cbus.cp.z;
                txKylinMsg.cbus.cv.z = 0;

                txKylinMsg.cbus.gp.c = GraspOp;
                txKylinMsg.cbus.gv.c = 10000;
            }
            if (finishGraspOpFlag == true)
            {
                if (boxNum == 6)
                {
                    coutLogicFlag = INT_MAX;
                    videoMove_PutBox2toBox1();
                }
                if (finish_PutBox2toBox1 == true || boxNum != 6)
                {
                    lastWs = workState;
                    rstRmp();
                    workState = 4; //进入下一阶段
                    txKylinMsg.cbus.cp.x = 0 + kylinOdomCalib.cbus.cp.x;
                    txKylinMsg.cbus.cv.x = 10;
                    txKylinMsg.cbus.cp.y = 0 + kylinOdomCalib.cbus.cp.y;
                    txKylinMsg.cbus.cv.y = 10;
                    txKylinMsg.cbus.gp.e = GraspTp;
                    txKylinMsg.cbus.gv.e = 10;
                }
            }

            break;
        case 4:
            //先后退
            if (finishBackMoveFlag == false)
            {
                detection_mode = 0;
                txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
                txKylinMsg.cbus.cp.x = 0;
                txKylinMsg.cbus.cv.x = 0;
                txKylinMsg.cbus.cp.y = -200;
                txKylinMsg.cbus.cv.y = 500;
                txKylinMsg.cbus.cp.z = 0;
                txKylinMsg.cbus.cv.z = 0;
		if(boxNum == 1 || boxNum == 4)
		{
			txKylinMsg.cbus.gp.e = GraspBw - 100 - kylinMsg.cbus.gp.e;
			txKylinMsg.cbus.gv.e = GRASPSPEED;
		}
		else
		{
			txKylinMsg.cbus.gp.e = GraspBw - 100 - kylinMsg.cbus.gp.e;
			txKylinMsg.cbus.gv.e = 0;
		}
                
                txKylinMsg.cbus.gp.c = GraspOp;
                txKylinMsg.cbus.gv.c = 8000;
            }

            if(finishBackMoveFlag == true && finishAbsoluteMoveFlag_BackOrigin == false)
            {
                ramp = genRmp();
                workState4_Num++;
                lineflag++;
                detection_mode = 0; //关闭视觉
                coutLogicFlag = 12;
                txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
                txKylinMsg.cbus.cp.x = 0 + kylinOdomCalib.cbus.cp.x;
                if (lineflag < 500) //4.6 TODO: forbidden the x-yaml move, time needs to debug
                    txKylinMsg.cbus.cv.x = 0;
                else
                {
                    txKylinMsg.cbus.cv.x = XSPEED * ramp;
                }
                txKylinMsg.cbus.cp.y = 0 + kylinOdomCalib.cbus.cp.y;
                txKylinMsg.cbus.cv.y = YSPEED * ramp;
                txKylinMsg.cbus.cp.z = 0 + kylinOdomCalib.cbus.cp.z;
                txKylinMsg.cbus.cv.z = ZSPEED;
                txKylinMsg.cbus.gp.e = GraspBw - 70 - kylinMsg.cbus.gp.e;
                txKylinMsg.cbus.gv.e = GRASPSPEED;
                txKylinMsg.cbus.gp.c = GraspOp;
                txKylinMsg.cbus.gv.c = 0;
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
                workState4_Num = 0, workState3_Num = 0, workState2_Num = 0, workState1_Num = 0, workState0_Num = 0;
                //coutLogicFlag = 0;
                boxNum++;
            }
            break;
        default:
            break;
        }
        if (boxNum == 7)
        {
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
/*
finishDetectBoxFlag_PutBox = false;
finishFixedUltrasonicFlag_1_PutBox = false;
finish_LR_UltrasonicFlag_PutBox = false;
finishFixedUltrasonicFlag_2_PutBox = false;
*/

//One Three boxes
void videoMove_PutBox()
{
    if (finishDetectBoxFlag_PutBox == false)
    {
        ramp = genRmp();
        coutLogicFlag_PutBox = 9.1;
        detection_mode = 1; //打开视觉,检测矩形
        //cout<<"detection_mode"<<(int)detection_mode<<endl;
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

        txKylinMsg.cbus.cp.x = tx - 60;
        txKylinMsg.cbus.cv.x = XSPEED * ramp;
        txKylinMsg.cbus.cp.y = tz;
        txKylinMsg.cbus.cv.y = YSPEED * ramp;
        txKylinMsg.cbus.cp.z = ry * 3141.592654f / 180;
        txKylinMsg.cbus.cv.z = ZSPEED;
        txKylinMsg.cbus.gp.e = (GraspBw + GraspTp)/2.0 - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = 0;
        txKylinMsg.cbus.gp.c = GraspCl;
        txKylinMsg.cbus.gv.c = 0;
    }
    //fixed Ultrasonic
    if (finishDetectBoxFlag_PutBox == true && finishFixedUltrasonicFlag_1_PutBox == false)
    {
        coutLogicFlag_PutBox = 9.2;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = sr04maf[SR04_IDX_F].avg; // - kylinMsg.cbus.cp.y;
        txKylinMsg.cbus.cv.y = 150;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = (GraspBw + GraspTp)/2.0 - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = 0;
        txKylinMsg.cbus.gp.c = GraspCl;
        txKylinMsg.cbus.gv.c = 0;
    }
    //left right Ultrasonic
    //add Grasp Flag
    if (finishFixedUltrasonicFlag_1_PutBox == true && finishGraspBwFlag_PutBox == false)
    {
        coutLogicFlag_PutBox = 9.21;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspBw - 30 - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = GRASPSPEED;
        txKylinMsg.cbus.gp.c = GraspCl; //抓子张开
        txKylinMsg.cbus.gv.c = 0;
    }
    if (finishGraspBwFlag_PutBox == true && finish_LR_UltrasonicFlag_PutBox == false)
    {
        coutLogicFlag_PutBox = 9.3;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = moveDistance;
        txKylinMsg.cbus.cv.x = 100;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspBw - 30 - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = GRASPSPEED;
        txKylinMsg.cbus.gp.c = 0; //抓子张开
        txKylinMsg.cbus.gv.c = 0;
    }
    //Put the forth box
    if (boxNum != 4)
    {
        if (finish_LR_UltrasonicFlag_PutBox == true && finishSlidTpFlag_PutBox == false)
        {
            coutLogicFlag_PutBox = 9.4;
            detection_mode = 0;                //关闭视觉
            txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
            txKylinMsg.cbus.cp.x = 0;
            txKylinMsg.cbus.cv.x = 0;
            txKylinMsg.cbus.cp.y = 0;
            txKylinMsg.cbus.cv.y = 0;
            txKylinMsg.cbus.cp.z = 0;
            txKylinMsg.cbus.cv.z = 0;
            // int GraspPosition = (GraspTp + GraspBw)/2.0;
            // if(boxNum == 1 || boxNum == 4)
            // {
            //     GraspPosition = GraspBw - 30;
            // }
            // if (boxNum == 2 || boxNum == 5)
            // {
            //     GraspPosition = GraspBw - 30 - 210;
            // }
            // if (boxNum == 3 || boxNum == 6)
            // {
            //     GraspPosition = GraspBw - 30 - 420;
            // }
            txKylinMsg.cbus.gp.e = GraspTp - kylinMsg.cbus.gp.e;
            txKylinMsg.cbus.gv.e = GRASPSPEED; //1000;
            txKylinMsg.cbus.gp.c = 0;          //抓子张开
            txKylinMsg.cbus.gv.c = 0;
        }
        if (finishSlidTpFlag_PutBox == true && finishFixedUltrasonicFlag_2_PutBox == false)
        {
            coutLogicFlag_PutBox = 9.5;
            detection_mode = 0;                //关闭视觉
            txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
            txKylinMsg.cbus.cp.x = 0;
            txKylinMsg.cbus.cv.x = 0;
            txKylinMsg.cbus.cp.y = sr04maf[SR04_IDX_F].avg;
            txKylinMsg.cbus.cv.y = 600;
            txKylinMsg.cbus.cp.z = 0;
            txKylinMsg.cbus.cv.z = 0;
            txKylinMsg.cbus.gp.e = 0;
            txKylinMsg.cbus.gv.e = 0;
            txKylinMsg.cbus.gp.c = 0; //抓子张开
            txKylinMsg.cbus.gv.c = 0;
        }
        if (finishFixedUltrasonicFlag_2_PutBox == true)
        {
            coutLogicFlag_PutBox = 9.6;
            finishAbsoluteMoveFlag_Put = true;
        }
    }
    else
    {
        if (finishGraspBwFlag_PutBox == true)
        {
            finishAbsoluteMoveFlag_Put = true;
        }
    }
}

void videoMove_PutBox2toBox1()
{
    //后退到550
    if (finishBackMoveFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.1;
        detection_mode = 0;
        //cout<<"detection_mode"<<(int)detection_mode<<endl;
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式

        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = -200;
        txKylinMsg.cbus.cv.y = 500;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = 0;
        txKylinMsg.cbus.gv.e = 0;
        txKylinMsg.cbus.gp.c = GraspOp;
        txKylinMsg.cbus.gv.c = 8000;
    }
    //降下抓子
    if (finishBackMoveFlag_PutBox2toBox1 == true && finishGraspBwFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.2;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspBw - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = GRASPSPEED;
        txKylinMsg.cbus.gp.c = 0; //抓子张开
        txKylinMsg.cbus.gv.c = 0;
    }
    //左右超声波对准
    if (finishGraspBwFlag_PutBox2toBox1 == true && finish_LR_UltrasonicFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.3;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = moveDistance;
        txKylinMsg.cbus.cv.x = 100;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspBw - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = GRASPSPEED;
        txKylinMsg.cbus.gp.c = 0; //抓子张开
        txKylinMsg.cbus.gv.c = 0;
    }
    //质心对准
    //squares detection finished, begin detect green area
    if (finish_LR_UltrasonicFlag_PutBox2toBox1 == true && finishDetectCentroidFlag_PutBox2toBox1 == false)
    {
        //finishFixedUltrasonicFlag = false;
        coutLogicFlag_PutBox2toBox1 = 10.4;
        detection_mode = 2;                //打开视觉,检测质心
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
        txKylinMsg.cbus.cp.x = tx;
        txKylinMsg.cbus.cv.x = 100;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspBw - 70 - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = GRASPSPEED;
        txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
        txKylinMsg.cbus.gv.c = 0;
    }
    //moble超声波引导
    if (finishDetectCentroidFlag_PutBox2toBox1 == true && finishMobleUltrasonicFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.5;
        detection_mode = 0;                //关闭视觉
        txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = sr04maf[SR04_IDX_M].avg;
        txKylinMsg.cbus.cv.y = 600;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspBw - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = GRASPSPEED;
        txKylinMsg.cbus.gp.c = 0; //抓子张开
        txKylinMsg.cbus.gv.c = 0;
    }
    //合拢抓子
    //begin grasp.
    if (finishMobleUltrasonicFlag_PutBox2toBox1 == true && finishGraspFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.6;
        //finishDetectCentroidFlag = false;   //clear the flag
        detection_mode = 0; //关闭视觉
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = 0; //(posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = 0;
        txKylinMsg.cbus.gp.c = GraspCl; //抓子合拢
        txKylinMsg.cbus.gv.c = 8000;
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
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspTp - kylinMsg.cbus.gp.e; //(posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f; // - kylinMsg.cbus.gp.e; //滑台升高到 30cm (相对位置控制模式，要做一个反馈)  //10: unstable control
        txKylinMsg.cbus.gv.e = GRASPSPEED;
        txKylinMsg.cbus.gp.c = GraspCl; //抓子合拢
        txKylinMsg.cbus.gv.c = 0;
    }
    if (finishSlidFlag_PutBox2toBox1 == true && finishFixedUltrasonicFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.8;
        detection_mode = 0;
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = sr04maf[SR04_IDX_F].avg; // - kylinMsg.cbus.cp.y;
        txKylinMsg.cbus.cv.y = 500;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = 0;
        txKylinMsg.cbus.gv.e = 0;
        txKylinMsg.cbus.gp.c = 0;
        txKylinMsg.cbus.gv.c = 0;
    }
    if (finishFixedUltrasonicFlag_PutBox2toBox1 == true && finishGraspOpFlag_PutBox2toBox1 == false)
    {
        coutLogicFlag_PutBox2toBox1 = 10.9;
        //finishDetectCentroidFlag = false;   //clear the flag
        detection_mode = 0; //关闭视觉
        txKylinMsg.cbus.fs &= ~(1u << 30);
        txKylinMsg.cbus.cp.x = 0;
        txKylinMsg.cbus.cv.x = 0;
        txKylinMsg.cbus.cp.y = 0;
        txKylinMsg.cbus.cv.y = 0;
        txKylinMsg.cbus.cp.z = 0;
        txKylinMsg.cbus.cv.z = 0;
        txKylinMsg.cbus.gp.e = GraspBw - kylinMsg.cbus.gp.e; //(posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f - kylinMsg.cbus.gp.e;
        txKylinMsg.cbus.gv.e = 0;
        txKylinMsg.cbus.gp.c = GraspOp - kylinMsg.cbus.gp.c; //抓子合拢
        txKylinMsg.cbus.gv.c = 8000;
    }
    if (finishGraspOpFlag_PutBox2toBox1 == true)
    {
        finish_PutBox2toBox1 = true;
    }
}

float Ultrasonic2Angle()
{
    float angle = 0;
    float frontDis = 150;
    float lrDis = 130;
    angle = atan((sr04maf[SR04_IDX_M].avg - sr04maf[SR04_IDX_R].avg - frontDis)/(lrDis));
    return angle;
}