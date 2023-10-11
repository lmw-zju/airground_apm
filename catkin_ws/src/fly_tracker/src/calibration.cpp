#include <opencv2/opencv.hpp>
#include <iostream>
 
using namespace cv;
using namespace std;
 
int main()
{
    // 定义棋盘格大小和个数
    Size boardSize(6, 8);
    int boardNum = 27;
    
    // 定义棋盘格角点坐标
    vector<vector<Point3f>> objPoints;
    /*定义了一个二维向量的变量objPoints，其中每个元素都是一个三维点(Point3f)的向量(vector)。*/
    /*其中objPoints中的每个元素都是一个由相机观察到的三维物体的点云，可以用于重建三维模型或者计算相机的内外参数等。*/
    vector<vector<Point2f>> imgPoints;
    /*用来存储棋盘格图案的角点坐标，用于计算相机的内外参数。*/
 
    // 生成棋盘格角点坐标
    vector<Point3f> obj;
    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            obj.push_back(Point3f(j*3, i*3, 0));//3cm:side length
        }
    }
    for (int i = 0; i < boardNum; i++)
    {
        objPoints.push_back(obj);
    }
    string prefix1="c-000",prefix2="c-00",filename;
    Size img_size;
    // 读取多个图片并提取角点
    for (int i = 0; i < boardNum; i++)
    {
        // 读取图片
        if(i<10) filename = "/home/lmw/Pictures/images/"+prefix1+to_string(i) + ".ppm";
        else  filename = "/home/lmw/Pictures/images/"+prefix2+to_string(i) + ".ppm";
        Mat image = imread(filename);
        resize(image, image, Size(), 0.5, 0.5);
        img_size.width = image.cols;
        img_size.height = image.rows;
       // cout<<"width is"<<img_size.width<<endl;
       // cout<<"height is"<<img_size.height<<endl;
        // 转换为灰度图
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
 
        // 提取角点
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners);
 
        if (found)
        {
            // 亚像素精细化
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
 
            // 存储角点
            imgPoints.push_back(corners);
 
            // 可视化角点
            drawChessboardCorners(image, boardSize, corners, found);
           // imshow("corners", image);
            //waitKey(0);
        }
        else cout<<"cannot find the corners!"<<endl;
    }
 
    // 标定相机
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objPoints, imgPoints, img_size, cameraMatrix, distCoeffs, rvecs, tvecs);
    /*
    calibrateCamera函数的参数包括：
    objectPoints：已知的3D空间点的坐标，类型为vector<vector<Point3f>>，其中Point3f是一个三维点的数据结构。
    imagePoints：对应的2D图像点的坐标，类型为vector<vector<Point2f>>，其中Point2f是一个二维点的数据结构。
    imgSize：图像的大小，类型为Size。
    cameraMatrix：相机的内部参数矩阵，类型为Mat。
    distCoeffs：相机的畸变系数，类型为Mat。
    rvecs：每个图像的旋转矢量，类型为vector<Mat>。
    tvecs：每个图像的平移矢量，类型为vector<Mat>。
    */
 
    // 打印标定结果
    //cout << "camera matrix:\n" << cameraMatrix << endl;
    //cout << "distortion coefficients:\n" << distCoeffs << endl;
    
   
    cout << "标定完成" << endl;
 
    //开始保存标定结果
    cout << "开始保存标定结果" << endl;
 
    cout << endl << "相机相关参数：" << endl;
    cout << "1.内外参数矩阵:" << endl;
    cout << "cameraMatrix大小：" << cameraMatrix.size() << endl;
    cout << "distortion coeffs大小：" << distCoeffs.size() << endl;
    // k1：径向畸变系数；
    // k2：径向畸变系数；
    // p1：切向畸变系数；
    // p2：切向畸变系数；
    // k3：径向畸变系数。
    cout << distCoeffs << endl;
    cout << endl << "图像相关参数：" << endl;
     Mat R;

    for (int i = 0;i < boardNum;i++)
    {
        cout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
        cout << rvecs[i] << endl;
        cout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
        Rodrigues(rvecs[i], R);//将旋转向量转换为相对应的旋转矩阵
        cout << R << endl;
        cout << "第" << i + 1 << "幅图像的平移向量：" << endl;
        cout << tvecs[i] << endl;
    }
 
    cout << "结果保存完毕" << endl;

    return 0;
}