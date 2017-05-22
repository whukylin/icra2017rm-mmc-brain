//Image processing
#include "opencv2/opencv.hpp"
#define DIF_CEN 190
#define PROP 0
#define CX   343   //To change to calibration parameter.
#define CY   320;   //the same with cameraMatrix.cx,cy
//#define _SHOW_PHOTO
//#define _SHOW_OUTPUT
//#define Camera_One
using namespace cv;
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

void findSquares( Mat src,const Mat& image, vector<vector<Point> >& squares );
void drawSquares( Mat& image, const vector<vector<Point> >& squares );
void LocationMarkes(const vector<vector<Point> >& squares);
void Calcu_attitude(Point3f world_pnt_tl,Point3f world_pnt_tr,Point3f world_pnt_br,Point3f world_pnt_bl,Point2f pnt_tl_src,Point2f pnt_tr_src,Point2f pnt_br_src,Point2f pnt_bl_src);
int Color_judge(Mat &src,int area);
void Sort_rect(vector<Point>& approx);
double angle( Point pt1, Point pt2, Point pt0 );
int Color_detect(Mat frame, int &diff_x, int &diff_y);
