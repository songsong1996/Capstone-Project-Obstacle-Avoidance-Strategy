//
// Created by songsong on 19-5-7.
//

#include "camera_opencv/contours_processor.h"
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>

using namespace std;
using namespace cv;


// vector<obstacle> obstacle_list;

double velocity;


double calDist(Point2d p1, Point2d p2)
{ return sqrt( pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0)); }

double calDist(Point3d p1, Point3d p2)
{ return sqrt( pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0)+pow(p1.z-p2.z,2.0)); }

void computeCentralPoint(vector<Point> &points,Point2d &central_point)
{
    auto itp=points.begin();
    central_point.x=0;central_point.y=0;

    while(itp!=points.end())
    {
        central_point.x+=(double)(*itp).x;
        central_point.y+=(double)(*itp).y;
        itp++;
    }
    central_point.x/=(double)points.size();
    central_point.y/=(double)points.size();
}




Mat match::match_contours2 (Mat &previous_top,Mat &previous_color, Mat &previous_depth, Mat &present_top,Mat &present_color,Mat &present_depth)
{
    preprocessImage(previous_top);
    preprocessImage(present_top);

//    Mat element = getStructuringElement(MORPH_RECT, Size(11, 11)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
//    //膨胀操作
//    dilate(previous_top, previous_top, element);
//    dilate(present_top, present_top, element);

    present_top.copyTo(present_tpp);
    previous_top.copyTo(previous_tpp);
    previous_color.copyTo(previous_color_tpp);
    previous_depth.copyTo(previous_depth_tpp);
    present_color.copyTo(present_color_tpp);
    present_depth.copyTo(present_depth_tpp);

    // vector<vector<Point>> contours_tp;
    vector<vector<Point>> contours_previous_tp, contours_present_tp,contours_previous, contours_present;
    vector<Vec4i> hierarchy;

    findContours(previous_top, contours_previous_tp, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(present_top, contours_present_tp, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    contours_previous=contours_filter(previous_top,contours_previous_tp,40);
    contours_present=contours_filter(present_top,contours_present_tp,40);

//    Mat previous_top_tp=convertTo3Channels(previous_top);
//    Mat present_top_tp=convertTo3Channels(present_top);
//
//    for (int i = 0; i < contours_previous.size(); i++) {
//        Scalar color =  Scalar(rand() % 255, rand() % 255, rand() % 255);
//        drawContours(previous_top_tp,contours_previous,i,color,3);
////        for (int j = 0; j < contours_previous[i].size() - 1; j++)
////            cv::line(previous_top_tp, contours_previous[i][j], contours_previous[i][j + 1], color,3);
//    }
//    cv::namedWindow("previous_top", WINDOW_NORMAL);
//    cv::imshow("previous_top",previous_top_tp);
//
//
//    for (int i = 0; i < contours_previous.size(); i++) {
//        Scalar color =  Scalar(rand() % 255, rand() % 255, rand() % 255);
//        drawContours(present_top_tp,contours_present,i,color,3);
////        for (int j = 0; j < contours_previous[i].size() - 1; j++)
////            cv::line(present_top_tp, contours_present[i][j], contours_present[i][j + 1], color,3);
//    }
//    cv::namedWindow("present_top", WINDOW_NORMAL);
//    cv::imshow("present_top",present_top_tp);
//    cv::waitKey(0);

    vector<obstacle> previous_obstacle,present_obstacle;
    initContourList( previous_obstacle,previous_color,previous_depth,previous_top,contours_previous);
    initContourList( present_obstacle,present_color,present_depth,present_top,contours_present);

 //   int tp=0;
//    if(present_obstacle.size()==1)
//    {
//        tp=1;
//        Mat tp=convertTo3Channels(present_depth);
//        rectangle(tp,present_obstacle[0].depth_contour.tl(),present_obstacle[0].depth_contour.br(),Scalar(255,0,255),3);
//        //drawContours(tp,present_obstacle[0].contours_point,0,Scalar(255,0,255),3);
//        imshow("depth",tp);
//        waitKey(0);
//        waitKey(0);
//        waitKey(0);
//    }
//    if(previous_obstacle.size()==1)
//        tp=1;

//    for(int i=0;i<previous_obstacle.size();i++)
//    {
//        rectangle(previous_color,previous_obstacle[i].depth_contour.br(),previous_obstacle[i].depth_contour.tl(),previous_obstacle[i].colorS,3);
//        draw_single(previous_top,previous_obstacle[i].contours_point,"top_previous", previous_obstacle[i].colorS);
//    }
//
//    imshow("color_previous",previous_color);
// //   imshow("top_previous",previous_top);
//    for(int i=0;i<present_obstacle.size();i++)
//    {
//        rectangle(present_color,present_obstacle[i].depth_contour.br(),present_obstacle[i].depth_contour.tl(),present_obstacle[i].colorS,3);
//        draw_single(present_top,present_obstacle[i].contours_point,"top_previous", present_obstacle[i].colorS,5);
//    }
//    imshow("color_present",present_color);
// //   imshow("top_present",present_top);
//
//    waitKey(0);

    delete_past(obstacle_list,present_obstacle);
    search_if_exist( present_obstacle,  obstacle_list);
    cout<<"obstacle_list.size()="<<obstacle_list.size()<<endl<<endl;
    Mat result=draw_contours(present_top,obstacle_list);
    return result;
}


int match::find_same(obstacle &obs,vector<obstacle> &obstacle_lists,int &is_exist)
{

    is_exist=0;
    if(obstacle_lists.size()==0)return 0;
    int num=0;
    int hash_min=100;
 //   imshow("color1",obs.color);
    //  waitKey(0);

    double d0=(double)obs.depth_contour.width/(double)obs.depth_contour.height;

    for(int i=0;i<obstacle_lists.size();i++)
    {
        int hash= HashMatch(obs.color,obstacle_lists[i].color,2);
        cout<<"hash="<<hash<<endl;
        if(hash<hash_min )
        {hash_min=hash;
            num=i;}
        else if(hash==hash_min)
        {
            num=-1;
        }
    }

    if(num!=-1)
    {
//        imshow("color_num_-1",obstacle_lists[num].color);
//        waitKey(0);
        double d2 =(double) obstacle_lists[num].depth_contour.width / (double) obstacle_lists[num].depth_contour.height;
        cout<<"d2="<<d2<<"  d1="<<d0<<endl;
        if (d2>d0*1.2 || d2<d0*0.8)
            num=-1;
    }
    if(num==-1) //hash==hash_min
    {

        double r0=0,g0=0,b0=0;
        for(int rr=0;rr<obs.color.rows;rr++)
            for(int cc=0;cc<obs.color.cols;cc++)
            {
                b0+=(double)obs.color.at<Vec3b>(rr,cc)[0];
                g0+=(double)obs.color.at<Vec3b>(rr,cc)[1];
                r0+=(double)obs.color.at<Vec3b>(rr,cc)[2];
            }

        b0/=(double)(obs.color.rows*obs.color.cols);
        g0/=(double)(obs.color.rows*obs.color.cols);
        r0/=(double)(obs.color.rows*obs.color.cols);


        double dist_min;
        cout<<"obstacle_num_num="<<obstacle_lists.size()<<endl;
        for(int i=0;i<obstacle_lists.size();i++)
        {
            double r=0,g=0,b=0;
            for(int rr=0;rr<obstacle_lists[i].color.rows;rr++)
                for(int cc=0;cc<obstacle_lists[i].color.cols;cc++)
                {
                    b+=(double)obstacle_lists[i].color.at<Vec3b>(rr,cc)[0];
                    g+=(double)obstacle_lists[i].color.at<Vec3b>(rr,cc)[1];
                    r+=(double)obstacle_lists[i].color.at<Vec3b>(rr,cc)[2];
                }

            b/=(double)(obstacle_lists[i].color.rows*obstacle_lists[i].color.cols);
            g/=(double)(obstacle_lists[i].color.rows*obstacle_lists[i].color.cols);
            r/=(double)(obstacle_lists[i].color.rows*obstacle_lists[i].color.cols);

            double  dist=calDist(Point3d(b,g,r),Point3d(b0,g0,r0));
            cout<<"dist="<<dist<<endl;

            if(i==0)
            {
                dist_min=dist;
                num=0;
            }
            if(  dist_min>dist  )
            {
                num=i;
                dist_min=dist;
            }
        }
//        imshow("color_match_dist",obstacle_lists[num].color);
//        waitKey(0);


    }

    if (hash_min<30)
    {
        cout<<"num="<<num<<endl;
        cout<<endl;
//        imshow("color2",obstacle_lists[num].color);
//        waitKey(0);
        is_exist=1;
    }
    return num;
}


void match::delete_past( vector<obstacle> &obstacle_lists,vector<obstacle> &contours )
{
    for(  vector<obstacle>::iterator it=obstacle_lists.begin();it!=obstacle_lists.end();)
    {
        int is_exist=0;
        int num;
        num=find_same(*it,contours,is_exist);

        if(!is_exist)
        {
            it=obstacle_lists.erase(it);
        }
        else
            it++;
    }
    cout<<obstacle_lists.size()<<endl<<"delete finish"<<endl<<"-------"<<endl;
}




void match::search_if_exist(vector<obstacle> &contours,  vector<obstacle> &obstacle_lists)
{
//    draw_contours(present_tpp,obstacle_list);
    vector<obstacle> obstacle_lists_tp;
    for(int i=0;i<obstacle_lists.size();i++)
    {obstacle_lists_tp.push_back(obstacle_lists.at(i));}

    for(  vector<obstacle>::iterator it=contours.begin();it!=contours.end();)
    {
        int is_exist=0;
        int num=0;
        num=find_same(*it,obstacle_lists,is_exist);

        if(is_exist==1)
        {
            obstacle_lists_tp[num].contours_point.assign( it->contours_point.begin(),   it->contours_point.end()    ) ;
            computeCentralPoint( obstacle_lists_tp[num].contours_point,obstacle_lists_tp[num].centralPoint);
            obstacle_lists_tp[num].position_list.push_back(obstacle_lists_tp[num].centralPoint);
            obstacle_lists_tp[num].color=it->color;
            obstacle_lists_tp[num].depth_contour=it->depth_contour;

            obstacle_lists_tp[num].kf.update(obstacle_lists_tp[num].centralPoint);
            obstacle_lists_tp[num].predict_pt=obstacle_lists_tp[num].kf.get_predict_pt();

//            double dist_tp=calDist(obstacle_lists_tp[num].position_list.at(obstacle_lists_tp[num].position_list.size()-1),obstacle_lists_tp[num].position_list.at(obstacle_lists_tp[num].position_list.size()-2));
//            //////////////////////////////////////////////////
//            if (dist_tp<abs(30+velocity) && dist_tp>abs(30-velocity))
//            {
//                obstacle_lists_tp[num].is_dynamic=true;
//            } else
//            {
//                obstacle_lists_tp[num].is_dynamic=false;
//            }
        }
        else
        {
            //add new obstacles
            obstacle tp(it->contours_point,it->depth_contour, it->color);
            obstacle_lists_tp.push_back(tp);
        }
        it++;
    }
    obstacle_lists.assign(obstacle_lists_tp.begin(),obstacle_lists_tp.end());

    //  draw_contours(present_tpp,obstacle_lists);

//    obstacle_lists.empty();
//
//    for(int i=0;i<obstacle_lists_tp.size();i++)
//    {obstacle_lists.push_back(obstacle_lists_tp.at(i));}
    cout<<obstacle_lists.size()<<endl<<"match finish"<<endl<<"-----"<<endl;
}


Mat match::draw_contours(Mat& present_top, vector<obstacle> &obstacle_lists)
{
    Mat top_tp;
    top_tp=convertTo3Channels(present_top);

//    cv::namedWindow("contours",WINDOW_NORMAL);
//    imshow("contours", top_tp);
//    waitKey(0);

    double size=obstacle_lists.size()*1.0;
    for(int i=0;i<obstacle_lists.size();i++)
    {
        Scalar color = Scalar(255.0/(size+5)*(i+5), 255.0/(size+5)*(i+5), 0);

        if(obstacle_lists[i].position_list.size()>1)
        {
            circle(top_tp, obstacle_lists[i].kf.get_predict_pt(), 5, Scalar(0, 255, 0),
                   3);    //predicted point with green
            circle(top_tp, obstacle_lists[i].position_list[obstacle_lists[i].position_list.size() - 1], 5,
                   Scalar(255, 0, 0), 3); //current position with red
        } else
        {
            circle(top_tp, obstacle_lists[i].position_list[obstacle_lists[i].position_list.size() - 1], 5,
                   Scalar(255, 0, 0), 3); //current position with red
        }

        for(int j=0;j<obstacle_lists.at(i).contours_point.size()-1;j++)
        {
            // drawContours(top_tp,obstacle_lists.at(i).contours_point,i,Scalar(100,100,100),3);
            cv::line(top_tp, obstacle_lists.at(i).contours_point[j],obstacle_lists.at(i).contours_point[j+1], color,3);
        }
    }
//    cv::namedWindow("contours",WINDOW_NORMAL);
//    imshow("contours", top_tp);
//    waitKey(20);

    return top_tp;
    //cout<<"finish"<<endl<<endl;
}

vector<vector<Point>> match::contours_filter(Mat &image,vector<vector<Point>> &unfiltered,double arealength)
{
    vector<vector<Point>> filtered;

    //  cout<<image.rows<<" "<<image.cols<<endl;

//original
//    for(int i=0;i< unfiltered.size();i++)
//    {
//        if (contourArea(unfiltered[i])>=arealength*arealength)
//            filtered.push_back(unfiltered[i]);
//        else if(boundingRect(unfiltered[i]).height>arealength ||  boundingRect(unfiltered[i]).width>arealength  )
//            filtered.push_back(unfiltered[i]);
//        else
//        {
//            cv::Rect r0= cv::boundingRect(cv::Mat(unfiltered[i]));
//            //  rectangle(image,r0.tl(),r0.br(),cv::Scalar(100,100,100),5,8);
////            cv::namedWindow("tpp",WINDOW_NORMAL);
////            imshow("tpp",image);
////            cv::waitKey(0);
//            for(int r=r0.y;r<r0.y+r0.height;r++)
//                for (int c = r0.x; c < r0.x + r0.width; c++)
//                    image.at<uchar>(r, c) = 0;
//        }
//    }

    for(int i=0;i< unfiltered.size();i++)
    {
        Point2d central;
        computeCentralPoint(unfiltered[i],central);
        if (contourArea(unfiltered[i])<=arealength*arealength)
        {
            cv::Rect r0= cv::boundingRect(cv::Mat(unfiltered[i]));
            //  rectangle(image,r0.tl(),r0.br(),cv::Scalar(100,100,100),5,8);
//            cv::namedWindow("tpp",WINDOW_NORMAL);
//            imshow("tpp",image);
//            cv::waitKey(0);
            for(int r=r0.y;r<r0.y+r0.height;r++)
                for (int c = r0.x; c < r0.x + r0.width; c++)
                    image.at<uchar>(r, c) = 0;
        }
        else if(boundingRect(unfiltered[i]).height<arealength and  boundingRect(unfiltered[i]).width<arealength  )
        {
            cv::Rect r0= cv::boundingRect(cv::Mat(unfiltered[i]));
            //  rectangle(image,r0.tl(),r0.br(),cv::Scalar(100,100,100),5,8);
//            cv::namedWindow("tpp",WINDOW_NORMAL);
//            imshow("tpp",image);
//            cv::waitKey(0);
            for(int r=r0.y;r<r0.y+r0.height;r++)
                for (int c = r0.x; c < r0.x + r0.width; c++)
                    image.at<uchar>(r, c) = 0;

        }
//        else if(central.x<20 || central.x>620 )
//        {
//            cv::Rect r0= cv::boundingRect(cv::Mat(unfiltered[i]));
//            //  rectangle(image,r0.tl(),r0.br(),cv::Scalar(100,100,100),5,8);
////            cv::namedWindow("tpp",WINDOW_NORMAL);
////            imshow("tpp",image);
////            cv::waitKey(0);
//            for(int r=r0.y;r<r0.y+r0.height;r++)
//                for (int c = r0.x; c < r0.x + r0.width; c++)
//                    image.at<uchar>(r, c) = 0;
//        }
        else
        {
            filtered.push_back(unfiltered[i]);
        }
    }


    cout<<"finish"<<endl;
//    cv::namedWindow("tpp",WINDOW_NORMAL);
//    imshow("tpp",image);
//    cv::waitKey(0);
    return filtered;
}





Mat match::convertTo3Channels( Mat &binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows, binImg.cols, CV_8UC3);
    vector<Mat> channels;
    for (int i = 0; i < 3; i++)
    { channels.push_back(binImg); }
    merge(channels, three_channel);
    return three_channel;
}

//match::match(vector<string> file_list_tp)
//{
//    file_list.empty();
//    obstacle_list.empty();
//    file_list=file_list_tp;
//
////    for(int i=0;i<file_list.size();i++)
////    {
////        Mat image;
////        image=imread(file_list[i],IMREAD_UNCHANGED);
////        Mat element = getStructuringElement(MORPH_RECT, Size(9,9)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
////        //膨胀操作
////        dilate(image, image, element);
////        vector<vector<Point>> contours;
////        vector<Vec4i> hierarchy;
////        findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
////        vector<vector<Point>> contours_tp;
////        contours_tp=contours_filter(image,contours,40);
////        imwrite("../data/top_view_dilate/"+ to_string(i+1)+".png",image);
////    }
//
//
//    for(int i=0;i<file_list.size()-1;i++)
//    {
//        Mat previous,present;
//        previous= imread(file_list.at(i),IMREAD_UNCHANGED );
//        present=imread(file_list.at(i+1),IMREAD_UNCHANGED );
//        match_contours(previous,  present) ;
//    }
//}


match::match(vector<string> color_list,vector<string> depth_list,vector<string> top_view)
{
    file_list=top_view;

//    for(int i=0;i<file_list.size();i++)
//    {
//        Mat image;
//        image=imread(file_list[i],IMREAD_UNCHANGED);
//        Mat element = getStructuringElement(MORPH_RECT, Size(9,9)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
//        //膨胀操作
//        dilate(image, image, element);
//        vector<vector<Point>> contours;
//        vector<Vec4i> hierarchy;
//        findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//        vector<vector<Point>> contours_tp;
//        contours_tp=contours_filter(image,contours,40);
//        imwrite("../data/top_view_dilate/"+ to_string(i+1)+".png",image);
//    }


    for(int i=0;i<file_list.size()-1;i++)
    {
        Mat previous,present;//8uc1
        Mat color_previous,color_present;//8uc3
        Mat depth_previous,depth_present;//16uc1

        previous= imread(file_list.at(i),IMREAD_UNCHANGED );
        present=imread(file_list.at(i+1),IMREAD_UNCHANGED );

        color_previous=imread(color_list[i],IMREAD_UNCHANGED);
        color_present=imread(color_list[i+1],IMREAD_UNCHANGED);

        depth_previous=imread(depth_list[i],IMREAD_UNCHANGED);
        depth_present=imread(depth_list[i+1],IMREAD_UNCHANGED);

        match_contours2(previous, color_previous,depth_previous,present,color_present,depth_present);
    }
}




void match::draw_single(Mat &image,vector<Point> &points,string windowname, Scalar color,int mode)
{
//    if (mode!=0)
//        color=Scalar(0,0,255);
//    else
//        color=Scalar(255,255,0);
    Mat tp;
    tp=convertTo3Channels(image);
    for(int i=0;i<points.size()-1;i++)
    {
        // circle(image,Point(20,20),10,color,2);
        line(tp,points[i],points[i+1],color,3);
    }
    namedWindow(windowname,WINDOW_NORMAL);
    cv::imshow(windowname,tp);
    cv::waitKey(0);
}



void match::delete_same(vector<obstacle> &obstacle_lists)
{
    vector<obstacle> tp;
    tp.assign(obstacle_lists.begin(),obstacle_lists.end());
    for(  vector<obstacle>::iterator it=tp.end();it!=tp.begin();)
    {
        bool is_exist=false;
        double match_min=100;
        double match_max=0;
        int num=0;
        draw_single(present_tpp,(*it).contours_point,"1",Scalar(255,255,0));
        for(int i=0;i<obstacle_lists.size();i++)
        {
            double match_value = matchShapes( (*it).contours_point, obstacle_lists[i].contours_point,CONTOURS_MATCH_I1, 0);
            double area=contourArea((*it).contours_point)>contourArea(obstacle_lists[i].contours_point)?
                        contourArea(obstacle_lists[i].contours_point)/contourArea((*it).contours_point):contourArea((*it).contours_point)/contourArea(obstacle_lists[i].contours_point);

            if(match_value!=0)
            {
                cout<<to_string(match_value)<<"  "<<to_string(area)<<"  "<<to_string(match_value/(area*area))<<"  "<<endl;
                if(match_value<3 && area>0.5 && match_value/(area*area)<5)
                {
                    is_exist = true;
                    double evaluate=area/(match_value/(area*area));
                    if (match_value > match_max) {
                        is_exist = true;
                        match_max = evaluate;
                        num = i;
                    }
                }
            }
        }
        double match_value = matchShapes(obstacle_lists[num].contours_point, (*it).contours_point, CONTOURS_MATCH_I1, 0);
        double area=contourArea((*it).contours_point)>contourArea(obstacle_lists[num].contours_point)?
                    contourArea(obstacle_lists[num].contours_point)/contourArea((*it).contours_point):contourArea((*it).contours_point)/contourArea(obstacle_lists[num].contours_point);
        cout<<to_string(match_value)<<"  "<<to_string(area)<<"  "<<to_string(match_value/(area*area))<<"  "<<is_exist<<endl;
        //     draw_single(present_tpp, obstacle_lists[num].contours_point, "2");
//        waitKey(0);
//        double area=contourArea((*it).contours_point)>contourArea(contours[num])?
//                    contourArea(contours[num])/contourArea((*it).contours_point):contourArea((*it).contours_point)/contourArea(contours[num]);

        //     cout<< match_min<<"  "<<area<<endl;
        //    if((match_min<5 and area>0.3 and match_min/(area*area)<5))  is_exist=true;
//        if(area>0.5)is_exist=true;


        if(is_exist)
        {
            it=obstacle_lists.erase(it);
            it=it-1;
        }
        else
            it--;
    }
}

void initContourList( vector<obstacle> &obstacle_lists,Mat &color,Mat &depth,Mat &topview,vector<vector<Point>> &contours)
{
    //get_depth_contour && delete_invalid_contours
    vector<vector<Point>> contours_tp;
    vector<Vec4i> hierarchy;
    depth=stressDepth2(depth);
    depth.convertTo(depth,CV_8UC1,255.0/65535.0);

//    Mat tpp;
//    color.copyTo(tpp);
//
//    Mat top_tpp;
//    topview.copyTo(top_tpp);
    Mat tpp=convertTo3Channels(depth);
    findContours(depth, contours_tp, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    double max_area=0,max_area2=0;

    for(int i=0;i<contours_tp.size();i++)
    {
        drawContours(tpp,contours_tp,i,Scalar(rand()%255,rand()%255,rand()%255),3);
        if(contourArea(contours_tp[i])>max_area)
            max_area=contourArea(contours_tp[i]);
    }

    for(int i=0;i<contours_tp.size();i++)
    {
        if(contourArea(contours_tp[i])>max_area2 && contourArea(contours_tp[i])<max_area)
            max_area2=contourArea(contours_tp[i]);
    }
//    imshow("tpp",tpp);
//    waitKey(0);

    for( vector<vector<Point>>::iterator it=contours_tp.begin();it!=contours_tp.end();)
    {
        if (contourArea((*it))<max_area2/2.0)
            it=contours_tp.erase(it);
        else
        {
            vector<Point> border;
            Rect tpr=boundingRect((*it));
            int id=-1;
            for(int i=0;i<contours.size();i++)
            {
                Rect tprr=boundingRect(contours[i]);
//                int thres=tpr.width*0.2;
//                if( (tprr.x<tpr.x+thres && tprr.x>tpr.x-thres) && (tprr.x+tprr.width<tpr.x+tpr.width+thres && tprr.x+tprr.width>tpr.x+tpr.width-thres)   )
//                {
//                    //  drawContours(top_tpp,contours,i,Scalar(rand()%255,rand()%255,rand()%255),3);
//                    id=i;
//                }
                int thres=tprr.width*0.2;
                if( (tpr.x<tprr.x+thres && tpr.x>tprr.x-thres) && (tpr.x+tpr.width<tprr.x+tprr.width+thres && tpr.x+tpr.width>tprr.x+tprr.width-thres)   )
                {
                    //  drawContours(top_tpp,contours,i,Scalar(rand()%255,rand()%255,rand()%255),3);
                    id=i;
                }
            }
            if(id!=-1)
            {
                //Mat tp_m=color(Range(tpr.y, tpr.y+tpr.height), Range(tpr.x, tpr.x+tpr.width));
                Mat tp_m=getRange(color,tpr.y, tpr.y+tpr.height,tpr.x, tpr.x+tpr.width);
                obstacle tp_o(contours[id],tpr,tp_m);
                obstacle_lists.push_back(tp_o);
                //  drawContours(tpp,contours_tp,it-contours_tp.begin(),Scalar(rand()%255,rand()%255,rand()%255),3);

            }
            it++;
        }
    }

    tpp=convertTo3Channels(depth);
    for(int i=0;i<contours_tp.size();i++)
    {
        drawContours(tpp,contours_tp,i,Scalar(rand()%255,rand()%255,rand()%255),3);
    }
//    imshow("tpp",tpp);
//    waitKey(0);

//    if(obstacle_lists.size()==1)
//    {
//        Mat top_tpp;
//        topview.copyTo(top_tpp);
//        imshow("top_tp",top_tpp);
//        waitKey(0);
//        waitKey(0);
//        waitKey(0);
//    }
//    imshow("tppppppppp", tpp);
//    imshow("topviewppp",top_tpp);
//    waitKey(0);
}
//void ContoursRemoveNoise(double pArea,Mat &image)
//{
//    int i,j;
//    int color = 1;
//    int nHeight = image.rows;
//    int nWidth = image.cols;
//
//    for (i = 0; i < nWidth; ++i)
//        for (j = 0; j < nHeight; ++j)
//        {
//            if ( !image.at<uchar>(i,j) )
//            {
//                //FloodFill each point in connect area using different color
//                floodFill(image,Point(i,j),Scalar(color));
//                color++;
//            }
//        }
//
//    int ColorCount[255] = { 0 };
//    for (i = 0; i < nWidth; ++i)
//    {
//        for (j = 0; j < nHeight; ++j)
//        {
//            //caculate the area of each area
//            if (getPixel(i,j) != 255)
//            {
//                ColorCount[getPixel(i,j)]++;
//            }
//        }
//    }
//    //get rid of noise point
//    for (i = 0; i < nWidth; ++i)
//    {
//        for (j = 0; j < nHeight; ++j)
//        {
//            if (ColorCount[getPixel(i,j)] <= pArea)
//            {
//                setPixel(i,j,WHITE);
//            }
//        }
//    }
//    for (i = 0; i < nWidth; ++i)
//    {
//        for (j = 0; j < nHeight; ++j)
//        {
//            if (getPixel(i,j) < WHITE)
//            {
//                setPixel(i,j,BLACK);
//            }
//        }
//    }
//}

void preprocessImage(Mat &image)
{

//    Mat element1 = getStructuringElement(MORPH_RECT, Size(2,2)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
//    //膨胀操作
//    dilate(image, image, element1);
//    cv::namedWindow("image_tp", WINDOW_NORMAL);
//    cv::imshow("image_tp",image);
//    cv::waitKey(0);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for(int i=0;i<contours.size();i++)
    {
        if(contourArea(contours[i])<=5) {
            cv::Rect r0 = cv::boundingRect(cv::Mat(contours[i]));
            //  rectangle(image,r0.tl(),r0.br(),cv::Scalar(100,100,100),5,8);
//            cv::namedWindow("tpp",WINDOW_NORMAL);
//            imshow("tpp",image);
//            cv::waitKey(0);
            for (int r = r0.y; r < r0.y + r0.height; r++)
                for (int c = r0.x; c < r0.x + r0.width; c++)
                    image.at<uchar>(r, c) = 0;
        }
    }

//    cv::namedWindow("image_tp", WINDOW_NORMAL);
//    cv::imshow("image_tp",image);
//    cv::waitKey(0);

    Mat element = getStructuringElement(MORPH_RECT, Size(15,15)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
    //膨胀操作
    dilate(image, image, element);

//    cv::namedWindow("image_tp", WINDOW_NORMAL);
//    cv::imshow("image_tp",image);
//    cv::waitKey(0);
}


Point2d linearPredict(obstacle tp)
{
    //only if position_list.size()>=2
    Point p1=tp.position_list[tp.position_list.size()-1];
    Point p2=tp.position_list[tp.position_list.size()-2];
    Point2d result;
    result.x=(double)(2*p1.x-p2.x);
    result.y=(double)(2*p1.y-p2.y);
    return result;
}



Mat stressDepth2(Mat &image)
{
    Mat result = Mat::zeros(image.rows, image.cols, CV_16UC1);
    cout<<image.at<uint16_t>(100,100)<<endl;
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++)
        {

            if(image.at<uint16_t>(i,j)>35000)result.at<uint16_t >(i,j)=0;
            else
                result.at<uint16_t>(i, j) = image.at<uint16_t>(i, j);
        }
    return result;
}

Mat getRange(Mat &img,int row1,int row2,int col1,int col2)
{
    Mat result=Mat::zeros(row2-row1+1,col2-col1+1,CV_8UC3);
    for(int i=row1;i<=row2;i++)
        for(int j=col1;j<=col2;j++)
            for(int k=0;k<3;k++)
        {
            result.at<Vec3b>(i-row1,j-col1)[k]=img.at<Vec3b>(i,j)[k];
        }
    return result;
}

