#include "main.h"
#include "RMVideoCapture.hpp"
#include "Markdetection.h"
#define COM_PORT 1 //"COM7"
#define BUF_LEN 256
#define TIMEOUT 30
#define FRAME_N 20000
using namespace cv;
using namespace std;

extern double ry,rz,rx;
extern double tx,ty,tz;
extern const char* wndname;

RMVideoCapture capture("/dev/video0", 3);
int exp_time = 62;
int gain = 30;
int brightness_ = 10;
int whiteness_ = 86;
int saturation_ = 60;

uint8_t exit_flag = 0;

FIFO_t rx_fifo;
uint8_t rx_buf[2][BUF_LEN];

ZGyroMsg_t zgyroMsg;
KylinMsg_t kylinMsg;
Sr04sMsg_t sr04sMsg;
PosCalibMsg_t posCalibMsg;

FIFO_t tx_fifo;
uint8_t tx_buf[2][BUF_LEN];
KylinMsg_t txKylinMsg;

static void Dnl_ProcZGyroMsg(const ZGyroMsg_t* zgyroMsg)
{
	//printf("*************************************ZGYRO********************************************\n");
	printf("id=%d,angle=%d,rate=%d\n", zgyroMsg->frame_id, zgyroMsg->angle, zgyroMsg->rate);
}

static void Dnl_ProcKylinMsg(const KylinMsg_t* kylinMsg)
{
        //printf("*************************************KYLIN********************************************\n");
        printf("id=%d,fs=%x,px=%d,py=%d,pz=%d,pe=%d,pc=%d,vx=%d,vy=%d,vz=%d,ve=%d,vc=%d\n", kylinMsg->frame_id, kylinMsg->cbus.fs,
                kylinMsg->cbus.cp.x, kylinMsg->cbus.cp.y, kylinMsg->cbus.cp.z, kylinMsg->cbus.gp.e, kylinMsg->cbus.gp.c,
                kylinMsg->cbus.cv.x, kylinMsg->cbus.cv.y, kylinMsg->cbus.cv.z, kylinMsg->cbus.gv.e, kylinMsg->cbus.gv.c);
}

static void Dnl_ProcSr04sMsg(const Sr04sMsg_t* sr04sMsg)
{
        //printf("*************************************KYLIN********************************************\n");
        printf("id=%d,fixed=%d,moble=%d\n", sr04sMsg->frame_id, sr04sMsg->fixed, sr04sMsg->moble);
}

static void Dnl_ProcPosCalibMsg(const PosCalibMsg_t* posCalibMsg)
{
        //printf("*************************************KYLIN********************************************\n");
        printf("id=%d,el=%d,eh=%d,cl=%d,ch=%d\n", posCalibMsg->frame_id, posCalibMsg->data.el, posCalibMsg->data.eh, posCalibMsg->data.cl, posCalibMsg->data.ch);
}

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
	// Push stream into fifo
	FIFO_Push(&rx_fifo, rx_buf[1], len);
	// Check if any message received
	if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_kylin, &kylinMsg)) {
		Dnl_ProcKylinMsg(&kylinMsg);
	}
	if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_sr04s, &sr04sMsg)) {
		//Dnl_ProcSr04sMsg(&sr04sMsg);
	}
	if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_zgyro, &zgyroMsg)) {
		//Dnl_ProcZGyroMsg(&zgyroMsg);
	}
	if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_pos_calib, &posCalibMsg)) {
		//Dnl_ProcPosCalibMsg(&posCalibMsg);
	}
}

void *KylinBotMsgPullerThreadFunc(void* param)
{
    while (exit_flag == 0) {
	      PullMsg();
	      usleep(1000);
	}
	return NULL;
}


void PushMsg()
{
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
    txKylinMsg.cbus.cp.y = 1000;
    txKylinMsg.cbus.cv.y = 1000 * Tri_Proc(&tri); //* Rmp_Calc(&rmp);
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
    txKylinMsg.cbus.cp.x = 1000;
    txKylinMsg.cbus.cv.x = 1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
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
    txKylinMsg.cbus.cp.y = -1000;
    txKylinMsg.cbus.cv.y = -1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
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
    txKylinMsg.cbus.cp.x = -1000;
    txKylinMsg.cbus.cv.x = -1000 * Tri_Proc(&tri); // * Rmp_Calc(&rmp);;
  }
  else {
    cnt = 0;
  }
  
  printf("cnt=%d\n", tri.cnt);
}



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
void setcamera()
{
#ifdef _SHOW_PHOTO
    namedWindow(wndname,1);
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
}

int main(int argc, char** argv)
{  
    setcamera();
    if(!capture.startStream())
    {
     cout<<"Open Camera failure.\n"; 
     return 1;
    }
    
       Mat frame;
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
    //printf("serial open error!\n");
    //return -1;
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
    LocationMarkes(squares);
    drawSquares(frame, squares);
    /*****************************/
    /*******send the object position to ARM*********/
    /***********************************************/
    //imshow(wndname, frame);
    int c = waitKey(1);

    t = ((double)getTickCount() - t)/getTickFrequency();
    //cout<<"time: "<<t<<" second"<<endl;

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
    
    txKylinMsg.cbus.cp.x = tx;
    txKylinMsg.cbus.cv.x = 500;
    txKylinMsg.cbus.cp.y = tz;
    txKylinMsg.cbus.cv.y = 800;
    txKylinMsg.cbus.cp.z = ry * 3141.592654f / 180;
    txKylinMsg.cbus.cv.z = 500;
    txKylinMsg.cbus.gp.e = ty;
    txKylinMsg.cbus.gv.e = 0;
    
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
  return 0;
}



