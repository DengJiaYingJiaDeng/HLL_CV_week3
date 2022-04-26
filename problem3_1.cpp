#include<iostream>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
using namespace std;
using namespace cv;

//筛选预处理
RotatedRect& adjustRec(RotatedRect& rec, const int mode)
{
     using std::swap;//交换

     float& width = rec.size.width;//宽度
     float& height = rec.size.height;//高度
     float& angle = rec.angle;//角度

     //纠正不是竖直的灯条轮廓
     if (mode == 0)
     {
          if (width < height)
          {
                 swap(width, height);
                 angle += 90.0;
          }
     }

     while (angle >= 90.0)
     {
         angle -= 180.0;
     }
     while (angle < -90.0)
     {
         angle += 180.0;
     }

     if (mode == 1)
     {
           if (angle >= 45.0)
           {
                 swap(width, height);

                 angle -= 90.0;
           }
            else if (angle < -45.0)
           {
                 swap(width, height);
                 angle += 90.0;
           }
      }
      return rec;

}
//卷积操作
int filter(int effective, int New,int delatmax)
{
    if ( ( New - effective > delatmax ) || ( effective - New > delatmax ))
    {
        New=effective;
        return effective;
    }
    else
    {
        New=effective;
        return New;
    }
}

int main()
{
    //读取图片
    Mat img = imread("E:\\armour.JPG",IMREAD_COLOR);
    namedWindow("img",0 );
    imshow("img",img);//展示

    //灰度图
    Mat gray;
    cvtColor(img,gray,COLOR_BGR2GRAY);//转灰度图
    namedWindow("gray",0 );
    imshow("gray",gray);//展示

    Mat thres;
    //转化为二值图
    threshold(gray,thres, 110, 255, THRESH_BINARY);
    namedWindow("thres",0 );
    imshow("thres",thres);

    //将原图RGB通道分离
    Mat splited[3];
    split(img, splited);

    //两图像相减（B通道减去R通道）
    Mat subtracted;
    subtract(splited[0], splited[2], subtracted);
    namedWindow("subtracted",0 );
    imshow("subtracted",subtracted);

    //将相减之后的的图片二值化
    threshold(subtracted,subtracted, 110, 255, THRESH_BINARY);
    Mat element1 = getStructuringElement(MORPH_RECT, Size(10, 10));
    //膨胀处理
    dilate(subtracted, subtracted, element1);
    namedWindow("dilate1",0 );
    imshow("dilate1",subtracted);

    //用原图灰度二值之后的图片与上原图通道相减二值膨胀之后的图片，提取出较为可信的候选区域后，再进行膨胀
    Mat ret;
    ret = thres & subtracted;
    Mat element2 = getStructuringElement(MORPH_RECT, Size(1, 1));
    dilate(ret, ret, element2);
    namedWindow("dilate2",WINDOW_NORMAL );
    imshow("dilate2",ret);

    //vector容器中放一个子容器，存放contours（检测到的轮廓）
    vector<vector<Point>>contours;//创建点集
    vector<Vec4i>hierarchy;//存放4维int向量
    findContours(ret,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));//寻找轮廓
    vector<RotatedRect> v;
    vector<RotatedRect> vRec;
    for (int i = 0;i <(int) contours.size();i++)
    {
         //求轮廓面积
         float Light_Contour_Area = contourArea(contours[i]);
         //去除较小轮廓fitllipse的限制条件
         if (Light_Contour_Area < 15 || contours[i].size() <= 10)
              continue;
         //椭圆拟合
         RotatedRect Light_Rec = fitEllipse(contours[i]);
         Light_Rec = adjustRec(Light_Rec,1);

          if (Light_Rec.angle > 10 )
              continue;
          // 根据矩形宽高比和面积比进行矩形筛选
          if (Light_Rec.size.width / Light_Rec.size.height > 1.5 || Light_Contour_Area / Light_Rec.size.area() < 0.5)
              continue;
           // 扩大灯柱的面积
          Light_Rec. size.height *= 2;
          Light_Rec.size.width *= 2;
          v.push_back(Light_Rec);
     }

     for (size_t i = 0; i < v.size(); i++)
     {
          for (size_t j = i + 1; (j < v.size()); j++)
          {
                //判断是否为相同灯条
                float Contour_angle = abs(v[i].angle - v[j].angle); //角度差
                if (Contour_angle >= 8)
                    continue;
                 //长度差比率
                 float Contour_Len1 = abs(v[i].size.height - v[j].size.height) / max(v[i].size.height, v[j].size.height);
                 //宽度差比率
                 float Contour_Len2 = abs(v[i].size.width - v[j].size.width) / max(v[i].size.width, v[j].size.width);
                 if (Contour_Len1 > 0.25 || Contour_Len2 > 0.25)
                     continue;

                 //储存Rec矩形
                 RotatedRect Rec;
                 Rec.center.x = (v[i].center.x + v[j].center.x) / 2.; //中点x坐标
                 Rec.center.y = (v[i].center.y + v[j].center.y) / 2.; //中点y坐标
                 Rec.angle = (v[i].angle + v[j].angle) / 2.; //角度
                 float h, w, yDiff, xDiff;
                 h = (v[i].size.height + v[j].size.height) / 2; //高度
                 // 宽度
                 w = sqrt((v[i].center.x - v[j].center.x) * (v[i].center.x - v[j].center.x) + (v[i].center.y - v[j].center.y) * (v[i].center.y - v[j].center.y));
                 float ratio = w / h; //匹配到的装甲板的长宽比
                 xDiff = abs(v[i].center.x - v[j].center.x) / h; //x差比率
                 yDiff = abs(v[i].center.y - v[j].center.y) / h; //y差比率
                 if (ratio < 1.0 || ratio > 5.0 || xDiff < 0.5 || yDiff > 2.0)
                       continue;
                 Rec.size.height = h;
                 Rec.size.width = w;
                 vRec.push_back(Rec);

                 //
                 Point2f point1;
                 Point2f point2;
                 point1.x=v[i].center.x;point1.y=v[i].center.y+20;
                 point2.x=v[j].center.x;point2.y=v[j].center.y-20;
                 int xmidnum = (point1.x+point2.x)/2;
                 int ymidnum = (point1.y+point2.y)/2;
                 //轮廓筛选完毕，将得到的数据输出处理
                 Rec.center.x = filter(Rec.center.x,xmidnum,30);
                 Rec.center.y = filter(Rec.center.y,ymidnum, 30);

                 rectangle(img, point1,point2, Scalar( 0, 255,0), 3);//画矩形
                 circle(img,Rec.center,5,Scalar(0,0,255),5);//在矩形中点画个小圆，用来标记矩形中点
                 cout<<"centerpoint:"<<Rec.center<<endl;

                 //使用putText函数标出矩形中点坐标
                 char centerpoint[100];
                 sprintf(centerpoint,"centerpoint(%0.1f,%0.1f)",Rec.center.x,Rec.center.y);
                 putText(img,centerpoint,point1, 0, 1, Scalar(0, 200, 0), 2,1);//将坐标显示在右下角
            }
      }

      namedWindow("after",0);
      imshow("after",img);

      waitKey(0);

      system("PAUSE");
      return 0;
}

