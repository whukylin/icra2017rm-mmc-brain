#include "main.h"
#include "RMVideoCapture.hpp"
#include "Markdetection.h"
#define COM_PORT 1 //"COM7"
#define BUF_LEN 256
#define TIMEOUT 30
#define FRAME_N 20000
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
#define SR04_MAF_LEN 10
Maf_t sr04maf[SR04_NUM];
float sr04buf[SR04_NUM][SR04_MAF_LEN];

FIFO_t tx_fifo;
uint8_t tx_buf[2][BUF_LEN];
KylinMsg_t txKylinMsg;



volatile bool finishAbsoluteMoveFlag = false;    //完成绝对位置移动
volatile bool finishDetectBoxFlag = false;       //完成检测盒子(小车到了检测不到盒子的位置)
volatile bool finishDetectCentroidFlag = false;  //完成质心检测
volatile bool finishMobleUltrasonicFlag = false; //超声波到达极限距离
volatile bool finishGraspFlag = false;           //抓子是否合拢
volatile bool finishSlidFlag = false;            //滑台是否达到指定高度
volatile bool finishFixedUltrasonicFlag = false;
volatile bool finish_LR_UltrasonicFlag = false;

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
		Maf_Proc(&sr04maf[0], sr04sMsg.fixed);
		Maf_Proc(&sr04maf[1], sr04sMsg.moble);
		Maf_Proc(&sr04maf[2], sr04sMsg.left);
		Maf_Proc(&sr04maf[3], sr04sMsg.right);
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
	
	FIFO_Init(&rx_fifo, rx_buf[0], BUF_LEN);
	FIFO_Init(&tx_fifo, tx_buf[0], BUF_LEN);
	Maf_Init(&sr04maf[0], sr04buf[0], SR04_MAF_LEN);
	Maf_Init(&sr04maf[1], sr04buf[1], SR04_MAF_LEN);
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

void *KylinBotMarkDetecThreadFunc(void *param)
{
	Mat frame;
	vector<vector<Point>> squares;
	int lostFlag = false;
	//KylinBotMsgPullerThreadFunc(NULL);
	//printf("hjhklhjllllllhkl\n");
	printf("exit_flag=%d\n",exit_flag);
	int lostCount = 0;
	int CountVframe=0;
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
		switch (detection_mode)
		{
			case 0: //do nothing
				//TODO:
				//imshow("IM",frame);
				cout << "detection_mode=" << (int)detection_mode << endl;
				break;
			case 1: //detect squares
				cout << "detection_mode=" << (int)detection_mode << endl;
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
				if (abs(tz) < 700 && (lostFlag == false)&&CountVframe>10)
				{ //Usue ultra sonic distance for controlling. Detection_mode will be changed in main.
					finishDetectBoxFlag = true;
					CountVframe=0;
				}
				else
				{
					finishDetectBoxFlag = false;
				}
				printf("tz=%lf\n",tz);
				break;
			case 2: //detect green area
				cout << "detection_mode=" << (int)detection_mode << endl;
				Color_detect(src, dif_x, dif_y);
				tx = 3 * (dif_x-DIF_CEN);
				// txKylinMsg.cbus.cp.x = 10 * dif_x;
				// txKylinMsg.cbus.cp.y = 0;
				// txKylinMsg.cbus.cp.z = 0;
				if (abs(tx-50)< 50) //number of pixels  
				{
					finishDetectCentroidFlag = true;
				}
				break;
			case 3: //follow line
				cout << "detection_mode=" << (int)detection_mode << endl;
				break;
			case 4: //follow line
				cout << "detection_mode=" << (int)detection_mode << endl;
				break;
			case 5: //follow line
				cout << "detection_mode=" << (int)detection_mode << endl;
				
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
	capture>>img;
	cout<<img.cols<<" "<<img.rows<<endl;
	namedWindow("Tuning",1);
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
	 
	while(1)
	{
		capture>>img;
		
		if(img.empty())
			continue;
		imshow("Tuning",img);
		int c=waitKey(20);
		if((char)c=='w')
			break;
	}
	destroyWindow("Tuning");
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
	cout << "workState: " << workState;
	cout << " finishAbsoluteMoveFlag:" << finishAbsoluteMoveFlag << endl;

}

KylinMsg_t kylinOdomCalib;
static uint32_t frame_cnt = 0;
uint8_t updateOdomCalib()
{
  uint32_t frame_id = kylinMsg.frame_id;
  while (frame_cnt < 10) {
    if (kylinMsg.frame_id != frame_id) {
      frame_id = kylinMsg.frame_id;
      frame_cnt++;
    }
    //cout << "waiting for new KylinMsg" << endl;
  }
  memcpy(&kylinOdomCalib, &kylinMsg, sizeof(KylinMsg_t));
  //cout << "kylinOdomCalib updated!" << endl;
  return 1;
}

KylinMsg_t kylinOdomError;
uint8_t updateOdomError()
{
  kylinOdomError.frame_id = kylinMsg.frame_id;
  kylinOdomError.cbus.cp.x = txKylinMsg.cbus.cp.x - kylinMsg.cbus.cp.x;// + kylinOdomCalib.cbus.cp.x;
  kylinOdomError.cbus.cp.y = txKylinMsg.cbus.cp.y - kylinMsg.cbus.cp.y;// + kylinOdomCalib.cbus.cp.y;
  kylinOdomError.cbus.cp.z = txKylinMsg.cbus.cp.z - kylinMsg.cbus.cp.z;// + kylinOdomCalib.cbus.cp.z;
  //cout << "ref.x=" << txKylinMsg.cbus.cp.x << ", fdb.x=" << kylinMsg.cbus.cp.x << endl;
  //cout << "ref.y=" << txKylinMsg.cbus.cp.y << ", fdb.y=" << kylinMsg.cbus.cp.y << endl;
  kylinOdomError.cbus.gp.e = txKylinMsg.cbus.gp.e - kylinMsg.cbus.gp.e;// + kylinOdomCalib.cbus.gp.e;
  kylinOdomError.cbus.gp.c = txKylinMsg.cbus.gp.c - kylinMsg.cbus.gp.c;// + kylinOdomCalib.cbus.gp.c;
  kylinOdomError.cbus.cv.x = txKylinMsg.cbus.cv.x - kylinMsg.cbus.cp.x;// + kylinOdomCalib.cbus.cv.x;
  kylinOdomError.cbus.cv.y = txKylinMsg.cbus.cv.y - kylinMsg.cbus.cv.y;// + kylinOdomCalib.cbus.cv.y;
  kylinOdomError.cbus.cv.z = txKylinMsg.cbus.cv.z - kylinMsg.cbus.cv.z;// + kylinOdomCalib.cbus.cv.z;
  kylinOdomError.cbus.gv.e = txKylinMsg.cbus.gv.e - kylinMsg.cbus.gv.e;// + kylinOdomCalib.cbus.gv.e;
  kylinOdomError.cbus.gv.c = txKylinMsg.cbus.gv.c - kylinMsg.cbus.gv.c;// + kylinOdomCalib.cbus.gv.c;
}

int main(int argc, char **argv)
{
	if(!setcamera())
	{
		cout<<"Setup camera failure. Won't do anything."<<endl;
		return   -1;
	}
	
	init();
	//uint32_t cnt = 0;
	Rmp_Config(&rmp, 50000);
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
	logicInit();          //逻辑控制初始化
	//workStateFlagPrint(); //打印当前状态
	int GraspTp=posCalibMsg.data.el;
	int GraspBw=posCalibMsg.data.eh;
	int GraspOp=posCalibMsg.data.cl;
	int GraspCl=posCalibMsg.data.ch;
	//sr04maf[1].avg moble
	//sr04maf[0].avg fixed
	//sr04maf[2].avg left
	//sr04maf[3].avg right
	while ((!exit_flag))  //&&(capture.read(frame)))
	{
		//cout << "ws: " << workState << endl;
		updateOdomError();
		
		//如果当前处于绝对位置控制模式,进入判断条件
		if (txKylinMsg.cbus.fs & (1u << 30)) //0xFF == 1111 1111   0x80000000
		{
			
			double absoluteDistance = pow(pow((kylinOdomError.cbus.cp.x), 2) + pow((kylinOdomError.cbus.cp.y), 2), 0.5);
			double absuluteAngle = abs(kylinOdomError.cbus.cp.z);
			//cout<<"absoluteDistance" << absoluteDistance << endl;
			//cout<<"absuluteAngle" << absuluteAngle << endl;
			if (absoluteDistance < 10 && absuluteAngle < 5.0f * PI / 2.0f)
			{
				finishAbsoluteMoveFlag = true; //完成绝对位置移动的控制标志位
			}
		}
		
		if((txKylinMsg.cbus.fs & (1u << 30)) == 0x00000000)
		{
			if(sr04maf[1].avg < 80)
			{
				finishMobleUltrasonicFlag = true;
			}
			if(kylinMsg.cbus.gp.c == posCalibMsg.data.ch)
			{
				finishGraspFlag = true;
			}
			if(kylinMsg.cbus.gp.e >= (posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f)
			{
				finishSlidFlag = true;
			}
			//fixed Ultra
			if(sr04maf[0].avg < 500)
			{
				finishFixedUltrasonicFlag = true;
			}
			if(sr04maf[2].avg > 600 && sr04maf[3].avg > 600)
			{
				finish_LR_UltrasonicFlag = true;
			}
			
		}
		//workStateFlagPrint(); //打印当前状态
		switch (workState)
		{
			case 0:
				//关闭视觉检测，小车在原点旋转90度，启动视觉检测（本阶段视觉关）仅仅只是//小车旋转
				detection_mode = 0;             //关闭视觉
				txKylinMsg.cbus.fs |= (1u << 30); //切换到绝对位置控制模式
				txKylinMsg.cbus.cp.x = 0 + kylinOdomCalib.cbus.cp.x;
				txKylinMsg.cbus.cv.x = 0;
				txKylinMsg.cbus.cp.y = 0 + kylinOdomCalib.cbus.cp.y;
				txKylinMsg.cbus.cv.y = 0;
				txKylinMsg.cbus.cp.z = 10+ kylinOdomCalib.cbus.cp.z; //1000 * PI / 2;// + kylinMsg.cbus.cp.z; //旋转90度
				txKylinMsg.cbus.cv.z = 1000;
				txKylinMsg.cbus.gp.e = GraspBw-100;
				txKylinMsg.cbus.gv.e = 1000;
				txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
				txKylinMsg.cbus.gv.c = 8000;
				if (finishAbsoluteMoveFlag == true)
				{
					workState = 1; //完成移动，进入下一阶段
					finishAbsoluteMoveFlag = false;
				}
				
				break;
			case 1:
				//begin detect squares
				if (finishDetectBoxFlag == false)
				{
					
					detection_mode = 1;                //打开视觉,检测矩形
					//cout<<"detection_mode"<<(int)detection_mode<<endl;
					txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
					
					txKylinMsg.cbus.cp.x = tx-60;
					txKylinMsg.cbus.cv.x = 500;
					txKylinMsg.cbus.cp.y = tz;
					txKylinMsg.cbus.cv.y = 800;
					txKylinMsg.cbus.cp.z = ry * 3141.592654f / 180;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = GraspBw - 100 + kylinMsg.cbus.gp.e;
					txKylinMsg.cbus.gv.e = 1000;
					txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
					txKylinMsg.cbus.gv.c = 8000;
				}
				//fixed Ultrasonic
				if(finishDetectBoxFlag == true && finishFixedUltrasonicFlag == false)
				{
					detection_mode = 0;                
					txKylinMsg.cbus.fs &= ~(1u << 30); 
					txKylinMsg.cbus.cp.x = 0;
					txKylinMsg.cbus.cv.x = 0;
					txKylinMsg.cbus.cp.y = sr04maf[0].avg - kylinMsg.cbus.cp.x;
					txKylinMsg.cbus.cv.y = 200;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = GraspBw - 100 + kylinMsg.cbus.gp.e;
					txKylinMsg.cbus.gv.e = 0;
					txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
					txKylinMsg.cbus.gv.c = 0;
				}
				//left right Ultrasonic
				if(finishFixedUltrasonicFlag = true && finish_LR_UltrasonicFlag == false)
				{
					detection_mode = 0;                
					txKylinMsg.cbus.fs &= ~(1u << 30); 
					txKylinMsg.cbus.cp.x = 10;
					txKylinMsg.cbus.cv.x = 200;
					txKylinMsg.cbus.cp.y = 0;
					txKylinMsg.cbus.cv.y = 0;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = GraspBw - 100 + kylinMsg.cbus.gp.e;
					txKylinMsg.cbus.gv.e = 0;
					txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
					txKylinMsg.cbus.gv.c = 0;
				}
				//squares detection finished, begin detect green area
				if (finish_LR_UltrasonicFlag == true && finishDetectCentroidFlag == false)
				{
					detection_mode = 2;                //打开视觉,检测质心
					txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
					txKylinMsg.cbus.cp.x = tx;
					txKylinMsg.cbus.cv.x = 200;
					txKylinMsg.cbus.cp.y = 0;
					txKylinMsg.cbus.cv.y = 0;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = GraspBw - 100 + kylinMsg.cbus.gp.e;
					txKylinMsg.cbus.gv.e = 1000;
					txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
					txKylinMsg.cbus.gv.c = 8000;
				}
				//image detection finished. only go forward
				if (finishDetectCentroidFlag == true && finishMobleUltrasonicFlag == false)
				{
					detection_mode = 0;                //关闭视觉
					txKylinMsg.cbus.fs &= ~(1u << 30); //切换到相对位置控制模式
					txKylinMsg.cbus.cp.x = 0;
					txKylinMsg.cbus.cv.x = 0;
					txKylinMsg.cbus.cp.y = sr04maf[1].avg;
					txKylinMsg.cbus.cv.y = 400;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = GraspBw + kylinMsg.cbus.gp.e;
					txKylinMsg.cbus.gv.e = 2000;
					txKylinMsg.cbus.gp.c = GraspOp; //抓子张开
					txKylinMsg.cbus.gv.c = 8000;
				}
				//begin grasp.
				if (finishMobleUltrasonicFlag == true && finishGraspFlag == false)
				{
					finishDetectCentroidFlag=false;   //clear the flag
					detection_mode = 0; //关闭视觉
					txKylinMsg.cbus.cp.x = 0;
					txKylinMsg.cbus.cv.x = 0;
					txKylinMsg.cbus.cp.y = 0;
					txKylinMsg.cbus.cv.y = 0;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = 0;//(posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f - kylinMsg.cbus.gp.e;
					txKylinMsg.cbus.gv.e = 0;
					txKylinMsg.cbus.gp.c = GraspCl; //抓子合拢
					txKylinMsg.cbus.gv.c = 8000;
				}
				//finish grasp. begin pull up.
				if (finishGraspFlag == true && finishSlidFlag == false)
				{
					detection_mode = 0; //关闭视觉
					txKylinMsg.cbus.cp.x = 0;
					txKylinMsg.cbus.cv.x = 0;
					txKylinMsg.cbus.cp.y = 0;
					txKylinMsg.cbus.cv.y = 0;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = (GraspBw + GraspTp)/2.0; //(posCalibMsg.data.el + posCalibMsg.data.eh) / 2.f; // - kylinMsg.cbus.gp.e; //滑台升高到 30cm (相对位置控制模式，要做一个反馈)
					txKylinMsg.cbus.gv.e = 1000;
					txKylinMsg.cbus.gp.c = GraspCl; //抓子合拢
					txKylinMsg.cbus.gv.c = 8000;
				}
				//抓到盒子并抬高了滑台, 进入下一届阶段，回到原点
				if (finishSlidFlag == true)
				{
					//workState = 2; //切换到下一阶段
				}
				
				break;
				case 2:
					//小车回到原点, 车头朝向前方
					detection_mode = 0;             //关闭视觉
					txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
					txKylinMsg.cbus.cp.x = 0;
					txKylinMsg.cbus.cv.x = 1000;
					txKylinMsg.cbus.cp.y = 0;
					txKylinMsg.cbus.cv.y = 1000;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 1000;
					txKylinMsg.cbus.gp.e = 300;
					txKylinMsg.cbus.gv.e = 400;
					txKylinMsg.cbus.gp.c = 2199;
					if (finishAbsoluteMoveFlag == true) //完成绝对位置控制模式
					{
						workState = 3; //切换到下一阶段
						finishAbsoluteMoveFlag == false;
					}
					break;
				case 3:
					//小车从原点达到基地区第一堆盒子的位置, 第一堆盒子的坐标 (0,3485)
					detection_mode = 0;             //关闭视觉
					txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
					txKylinMsg.cbus.cp.x = 0;
					txKylinMsg.cbus.cv.x = 1000;
					txKylinMsg.cbus.cp.y = 3485;
					txKylinMsg.cbus.cv.y = 1000;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 0;
					txKylinMsg.cbus.gp.e = 300;
					txKylinMsg.cbus.gv.e = 400;
					txKylinMsg.cbus.gp.c = 2199;
					if (finishAbsoluteMoveFlag == true) //完成绝对位置控制模式
					{
						//到达指定位置，放下盒子
						if (finishSlidFlag == false)
						{
							txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
							txKylinMsg.cbus.cp.x = 0;
							txKylinMsg.cbus.cv.x = 1000;
							txKylinMsg.cbus.cp.y = 3485;
							txKylinMsg.cbus.cv.y = 1000;
							txKylinMsg.cbus.cp.z = 0;
							txKylinMsg.cbus.cv.z = 0;
							txKylinMsg.cbus.gp.e = 0;
							txKylinMsg.cbus.gv.e = 400;
							txKylinMsg.cbus.gp.c = 2199;
						}
						//松开抓子
						if (finishSlidFlag == true && finishGraspFlag == false)
						{
							txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
							txKylinMsg.cbus.cp.x = 0;
							txKylinMsg.cbus.cv.x = 1000;
							txKylinMsg.cbus.cp.y = 3485;
							txKylinMsg.cbus.cv.y = 1000;
							txKylinMsg.cbus.cp.z = 0;
							txKylinMsg.cbus.cv.z = 1000;
							txKylinMsg.cbus.gp.e = 0;
							txKylinMsg.cbus.gv.e = 0;
							txKylinMsg.cbus.gp.c = 314;
						}
						if (finishGraspFlag == true)
						{
							workState = 4;                   //进入下一阶段
							finishAbsoluteMoveFlag == false; //完成本阶段的绝对位置控制
						}
					}
					break;
				case 4:
					//回原点
					detection_mode = 0;             //关闭视觉
					txKylinMsg.cbus.fs |= 1u << 30; //切换到绝对位置控制模式
					txKylinMsg.cbus.cp.x = 0;
					txKylinMsg.cbus.cv.x = 1000;
					txKylinMsg.cbus.cp.y = 0;
					txKylinMsg.cbus.cv.y = 1000;
					txKylinMsg.cbus.cp.z = 0;
					txKylinMsg.cbus.cv.z = 1000;
					txKylinMsg.cbus.gp.e = 0;
					txKylinMsg.cbus.gv.e = 0;
					txKylinMsg.cbus.gp.c = 314;
					if (finishAbsoluteMoveFlag == true)
					{
						workState = 0;                   //进入下一阶段
						finishAbsoluteMoveFlag == false; //完成本阶段的绝对位置控制
					}
					break;
				default:
					break;
		}
	}
	
	//if(capture.isOpened())
	//	capture.release();
	capture.closeStream();
	disconnect_serial();
	return 0;

}
