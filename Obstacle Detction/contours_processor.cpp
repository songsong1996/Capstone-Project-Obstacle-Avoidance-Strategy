//
// Created by songsong on 19-5-7.
//

#include "contours_processor.h"
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



vector<Mat> match::match_contours2 (Mat &previous_top,Mat &previous_color, Mat &previous_depth, Mat &present_top,Mat &present_color,Mat &present_depth)
{
    preprocessImage(previous_top);
    preprocessImage(present_top);
//    imshow("top",previous_top);
//    imshow("de",previous_depth);
//    waitKey(0);

//    Mat element = getStructuringElement(MORPH_RECT, Size(11, 11)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
//    //膨胀操作
//    dilate(previous_top, previous_top, element);
//    dilate(present_top, present_top, element);


//t
    present_top.copyTo(present_tpp);
    previous_top.copyTo(previous_tpp);
    previous_color.copyTo(previous_color_tpp);
    previous_depth.copyTo(previous_depth_tpp);
    present_color.copyTo(present_color_tpp);
    present_depth.copyTo(present_depth_tpp);

    vector<vector<Point>> contours_previous_tp, contours_present_tp,contours_previous, contours_present,contours_depth_previous,contours_depth_present;
    vector<Vec4i> hierarchy;

    findContours(previous_top, contours_previous_tp, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(present_top, contours_present_tp, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    findContours(previous_depth, contours_depth_previous, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    findContours(present_depth, contours_depth_present, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    contours_previous=contours_filter(previous_top,contours_previous_tp,40);
//    contours_present=contours_filter(present_top,contours_present_tp,40);




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




//t
    vector<obstacle> previous_obstacle,present_obstacle;
    initContourList( previous_obstacle,previous_color,previous_depth,previous_top,contours_previous_tp,contours_depth_previous);
    initContourList( present_obstacle,present_color,present_depth,present_top,contours_present_tp,contours_depth_present);



//    int tp=0;
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
//
//    for(int i=0;i<previous_obstacle.size();i++)
//    {
//        rectangle(previous_color,previous_obstacle[i].depth_contour.br(),previous_obstacle[i].depth_contour.tl(),previous_obstacle[i].colorS,3);
//        draw_single(previous_top,previous_obstacle[i].contours_point,"top_previous", previous_obstacle[i].colorS);
//    }
//
//    imshow("color_previous",previous_color);
//    //   imshow("top_previous",previous_top);
//    for(int i=0;i<present_obstacle.size();i++)
//    {
//        rectangle(present_color,present_obstacle[i].depth_contour.br(),present_obstacle[i].depth_contour.tl(),present_obstacle[i].colorS,3);
//        draw_single(present_top,present_obstacle[i].contours_point,"top_previous", present_obstacle[i].colorS,5);
//    }
//    imshow("color_present",present_color);
//    //   imshow("top_present",present_top);
//
//    waitKey(0);





//t
    delete_past(obstacle_list,present_obstacle);
    search_if_exist( present_obstacle,  obstacle_list);
    if_stop=if_stop_now(obstacle_list);

    present_color.copyTo(color_draw);
    depth_draw=convertTo3Channels(present_depth);
    for(int i=0;i<obstacle_list.size();i++)
    {
        rectangle(color_draw,obstacle_list[i].depth_contour.tl(),obstacle_list[i].depth_contour.br(),obstacle_list[i].colorS,2);
        rectangle(depth_draw,obstacle_list[i].depth_contour.tl(),obstacle_list[i].depth_contour.br(),obstacle_list[i].colorS,2);
    }

    cout<<"obstacle_list.size()="<<obstacle_list.size()<<endl<<endl;

    Mat result= draw_contours(present_top,obstacle_list);
    Mat color=draw_color(present_color, obstacle_list);
    Mat ground=draw_ground(present_color,present_depth);
    Mat optical=optical_flow(previous_color,present_color,previous_depth,present_depth);
    vector<Mat> rr;
    rr.push_back(result);
    rr.push_back(color);
    rr.push_back(ground);
    rr.push_back(optical);
    return rr;
}


int match::find_same(obstacle &obs,vector<obstacle> &obstacle_lists,int &is_exist)
{

    is_exist=0;
    if(obstacle_lists.size()==0)return 0;
    int num=0;
    int hash_min=100;
    imshow("color1",obs.color);
    imwrite("../data/mm/00.png",obs.color);
    waitKey(0);

    double dd;
    double d0=(double)obs.depth_contour.width/(double)obs.depth_contour.height;

    //hash_match
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


    //color_match
    if(num!=-1)
    {
        imshow("hash_match_-1",obstacle_lists[num].color);
        imwrite("../data/mm/01.png",obstacle_lists[num].color);
        waitKey(0);
        double d2 =(double) obstacle_lists[num].depth_contour.width / (double) obstacle_lists[num].depth_contour.height;
        cout<<"d2="<<d2<<"  d1="<<d0<<endl;

        d0=obs.depth_contour.width*obs.depth_contour.height;
        d2=obstacle_lists[num].depth_contour.width*obstacle_lists[num].depth_contour.height;

        if (d2>d0*1.2|| d2<d0*0.8)
            num=-1;
    }
    if(num==-1) //hash==hash_min
    {
        double dist_min;
        double d_min;
        cout<<"obstacle_num_num="<<obstacle_lists.size()<<endl;
        for(int i=0;i<obstacle_lists.size();i++)
        {

            double dist=calColorMean(obs.color,obstacle_lists[i].color);

            //   double ssim=1.0-SSIM(obs.color,obstacle_lists[i].color);
            double h=HashMatch(obs.color,obstacle_lists[i].color,2);

            cout<<"dist="<<dist<<endl;
            //   "         1-ssim="<<ssim<<endl;


            if(i==0)
            {
                d_min=dist*h;
                dist_min=dist;
                num=0;
                // dist_diff=1000;

            }
            if(  d_min>dist*h )
            {
                num=i;
                d_min=dist*h;
            }
            if(dist_min>dist)
            {
                dist_min=dist;
            }
            cout<<"ddd="<<dist*h<<endl;

        }
        dd=d_min;
        imshow("color_match_dist",obstacle_lists[num].color);
        imwrite("../data/mm/02.png",obstacle_lists[num].color);
        waitKey(0);

    }

    if(num!=-1)
    {
        double dist_diff=1000;
        for(int i=0;i<obstacle_lists.size()-1;i++) {
            double dist0 = calColorMean(obs.color, obstacle_lists[i].color);
            for (int j = i + 1; j < obstacle_lists.size(); j++) {
                double dist1 = calColorMean(obs.color, obstacle_lists[j].color);
                if(dist_diff<abs(dist1-dist0))
                    dist_diff=abs(dist1-dist0);
            }
        }
        if(dist_diff<30)
        {
            num=-1;
        }
    }
    if(num==-1)
    {

    }







//    if (hash_min<30)
//    {
//        cout<<"num="<<num<<endl;
//        cout<<endl;

    cout<<"dd="<<dd<<"----"<<endl;
    int rr1=obs.depth_contour.x+obs.depth_contour.width/2<previous_depth_tpp.cols/2?-1:1;
    int rr2=obstacle_lists[num].depth_contour.x+obstacle_lists[num].depth_contour.width/2<previous_depth_tpp.cols/2?-1:1;

    if(rr1*rr2>0)
    {
        imshow("color2",obstacle_lists[num].color);
        waitKey(0);
        is_exist=1;
    } else
    {
        num=-1;
        is_exist=0;
        return num;
    }

    if(dd>1150)
    {
        num=-1;
        is_exist=0;
        return num;
    }
    else
    {
        imshow("color2",obstacle_lists[num].color);
        waitKey(0);
        is_exist=1;
    }
    imwrite("../data/mm/03.png",obstacle_lists[num].color);

    //        is_exist=1;
//    }
    return num;
}



int match::find_same2(obstacle &obs,vector<obstacle> &obstacle_lists,int &is_exist)
{

    is_exist=0;
    if(obstacle_lists.size()==0)return 0;
    int num=0;
    int hash_min=100;
    imshow("color1",obs.color);
    imwrite("../data/mm/00.png",obs.color);
     waitKey(0);

    double dd;
    double d0=(double)obs.depth_contour.width/(double)obs.depth_contour.height;

    vector<double> hash_match,color_dist,ssim;
    for(int i=0;i<obstacle_lists.size();i++)
    {
        hash_match.push_back(HashMatch(obs.color,obstacle_lists[i].color,2));
        color_dist.push_back( calColorMean(obs.color, obstacle_lists[i].color)  );
        ssim.push_back( 1.0-SSIM(obs.color,obstacle_lists[i].color));
    }


    if(obstacle_lists.size()>1)
    {
        vector<int>value;
        vector<double> d1= normalize_list(hash_match);
        vector<double> d2= normalize_list(color_dist);
        vector<double> d3= normalize_list(ssim);
        double match_min=1;
        num=-1;
        for(int i=0;i<d1.size();i++)
        {
            double match=0.2*d1[i]+0.2*d2[i]+0.6*d3[i];
            cout<<"hash="<<hash_match[i]<<" "<<d1[i]<<"   color="<< color_dist[i]<<" "<<d2[i]<<"   ssim="<<ssim[i]<<" "<<d3[i]<<endl;
            cout<<"match="<<match<<endl;
            if(match<=match_min) {
                match_min = match;
                num=i;
            }
        }
        cout<<"match_min="<<match_min<<"---"<<endl;
         imshow("color2",obstacle_lists[num].color);
        imwrite("../data/mm/01.png",obstacle_lists[num].color);
      //   waitKey(0);
    } else
    {
        cout<<"num="<<num<<"  hash="<<hash_match[0]<<"   color="<< color_dist[0]<<"   ssim="<<ssim[0]<<"-----"<<endl;
           imshow("color2",obstacle_lists[num].color);
        imwrite("../data/mm/01.png",obstacle_lists[num].color);
       //    waitKey(0);
    }



    int rr1=obs.depth_contour.x+obs.depth_contour.width/2<previous_depth_tpp.cols/2?-1:1;
    int rr2=obstacle_lists[num].depth_contour.x+obstacle_lists[num].depth_contour.width/2<previous_depth_tpp.cols/2?-1:1;

    if(rr1*rr2>0)
    {
            imshow("color3",obstacle_lists[num].color);
        cout<<"is_exist="<<color_dist[num]*hash_match[num]<<"--------"<<endl;
        imwrite("../data/mm/02.png",obstacle_lists[num].color);
         waitKey(0);
        is_exist=1;

        if(color_dist[num]*hash_match[num]>1500)
        {
           cout<<"is_exist="<<color_dist[num]*hash_match[num]<<"--------"<<endl;
            num=-1;
            is_exist=0;
            return num;
        } else
        {
             cout<<"is_exist="<<color_dist[num]*hash_match[num]<<"--------"<<endl;
            imwrite("../data/mm/02.png",obstacle_lists[num].color);
            imshow("color3",obstacle_lists[num].color);

            waitKey(0);
            is_exist=1;
            return num;
        }

    } else
    {
        num=-1;
        is_exist=0;
        return num;
    }

}



double match::calColorMean(Mat &img0,Mat &img1)
{
    double r0=0,g0=0,b0=0;
    double num0=0;
    double dist_min;
    for(int rr=0;rr<img0.rows;rr++)
        for(int cc=0;cc<img0.cols;cc++)
        {
            if(  img0.at<Vec3b>(rr,cc)!=Vec3b(0,0,0) )
            {
                num0+=1.0;
                b0+=(double)img0.at<Vec3b>(rr,cc)[0];
                g0+=(double)img0.at<Vec3b>(rr,cc)[1];
                r0+=(double)img0.at<Vec3b>(rr,cc)[2];
            }
        }

    b0/=num0;
    g0/=num0;
    r0/=num0;


    double r=0,g=0,b=0;
    double num1=0;
    for(int rr=0;rr<img1.rows;rr++)
        for(int cc=0;cc<img1.cols;cc++)
        {
            if(  img1.at<Vec3b>(rr,cc)!=Vec3b(0,0,0) )
            {
                num1+=1.0;
                b+=(double)img1.at<Vec3b>(rr,cc)[0];
                g+=(double)img1.at<Vec3b>(rr,cc)[1];
                r+=(double)img1.at<Vec3b>(rr,cc)[2];
            }
        }

    b/=num1;
    g/=num1;
    r/=num1;

    double  dist=calDist(Point3d(b,g,r),Point3d(b0,g0,r0));
    return dist;
}

void match::delete_past( vector<obstacle> &obstacle_lists,vector<obstacle> &contours )
{
    for(  vector<obstacle>::iterator it=obstacle_lists.begin();it!=obstacle_lists.end();)
    {
        int is_exist=0;
        int num;
        num=find_same2(*it,contours,is_exist);

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
        num=find_same2(*it,obstacle_lists,is_exist);

        if(is_exist==1)
        {
            obstacle_lists_tp[num].contours_point.assign( it->contours_point.begin(),   it->contours_point.end()    ) ;
            computeCentralPoint( obstacle_lists_tp[num].contours_point,obstacle_lists_tp[num].centralPoint);
            obstacle_lists_tp[num].position_list.push_back(obstacle_lists_tp[num].centralPoint);
            obstacle_lists_tp[num].color=it->color;
            obstacle_lists_tp[num].depth_contour=it->depth_contour;
            obstacle_lists_tp[num].contours_point_depth=it->contours_point_depth;
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
            obstacle tp(it->contours_point,it->contours_point_depth, it->depth_contour, it->color);
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
      cv::namedWindow("contours",WINDOW_NORMAL);
      imshow("contours", top_tp);
       waitKey(0);
    //cout<<"finish"<<endl<<endl;
    return top_tp;
}

Mat match::draw_color(Mat& present_color, vector<obstacle> &obstacle_lists)
{
    Mat color;
    present_color.copyTo(color);
    for(int k=0;k<obstacle_lists.size();k++)
    {
        for(int i=obstacle_lists[k].depth_contour.y;i<obstacle_lists[k].depth_contour.y+obstacle_lists[k].depth_contour.height;i++ )
            for(int j=obstacle_lists[k].depth_contour.x;j<obstacle_lists[k].depth_contour.x+obstacle_lists[k].depth_contour.width;j++ )
            {
                if(  pointPolygonTest(obstacle_lists[k].contours_point_depth, Point2f(j, i), false) ==1 )
                {
                    for(int ii=0;ii<3;ii++)
                        color.at<Vec3b>(i,j)[ii]=obstacle_lists[k].colorS.val[ii];
                }
            }
    }

    Mat result;
    addWeighted(color, 0.4, present_color, 0.6, 0, result);
    return result;
}

Mat match::draw_ground(Mat& present_color, Mat& present_depth)
{

    Mat im1=Mat::zeros(present_color.size(),CV_8UC1);
    vector<Point> ground;
    ground.push_back(Point(524,180));
    ground.push_back(Point(0,present_color.rows));
    ground.push_back(Point(present_color.cols,present_color.rows));
    ground.push_back(Point(732,182));
    cv::fillConvexPoly(im1,ground,Scalar(255,255,255));
//    imshow("ground",im1);
//    waitKey(0);

    Mat result,result1;
    present_color.copyTo(result);
    for(int i=0;i<present_color.rows;i++)
        for(int j=0;j<present_color.cols;j++)
        {
            if( present_depth.at<ushort>(i,j)==0  && im1.at<uchar>(i,j)==255)
           // if( present_depth.at<ushort>(i,j)!=0 )
            {
                for(int k=0;k<3;k++)
                {
                    result.at<Vec3b>(i,j)[k]=255;
                }
            }
        }
//    imshow("rr",result);
//        waitKey(0);

    addWeighted(result, 0.4, present_color, 0.6, 0, result1);
    return result1;
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


bool match::if_stop_now(vector<obstacle> &obstacle_lists)
{
    bool flag=false;
    for(int i=0;i<obstacle_lists.size();i++)
    {
        if(600-obstacle_lists[i].position_list[obstacle_lists[i].position_list.size()-1].y<=250)
            flag=true;
    }

    if(obstacle_lists.size()==0)
    {
        int exist=0;
        for(int i=present_tpp.rows-250;i<present_tpp.rows;i++)
        {
            for (int j = 0; j < present_tpp.cols; j++) {
                if (present_tpp.at<uchar>(i, j) != 0) {
                    exist = 1;
                    break;
                }
            }
            if(exist==1)
                break;
        }
        if(exist==1)
            flag=true;
    }
    return flag;
}



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

    for(int i=18;i<file_list.size()-1;i++)
        //  for(int i=0;i<file_list.size()-1;i++)
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

        cout<<file_list[i]<<endl;
        file_num=i;
        vector<Mat> re=match_contours2(previous, color_previous,depth_previous,present,color_present,depth_present);
        string str=file_list[i].substr(file_list[i].find_last_of("/")+1,11);

        imwrite("../data/pcd/result/"+ str+".png",re[0]);
        imwrite("../data/pcd/color_result/"+ str+".png",re[1]);
        imwrite("../data/pcd/ground/"+ str+".png",re[2]);
        imwrite("../data/pcd/flow/"+ str+".png",re[3]);
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
//    namedWindow(windowname,WINDOW_NORMAL);
//    cv::imshow(windowname,tp);
//    cv::waitKey(0);
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

void process_Contours(Mat top_view ,Mat depth, vector<vector<Point>> &contours_top_view, vector<vector<Point>> &contours_depth)
{
    if(contours_depth.size()!=0) {
        for (int i = 0; i < contours_depth.size() - 1; i++) {
            for (int j = 0; j < contours_depth.size() - i - 1; j++) {
                if (contourArea(contours_depth[j]) < contourArea(contours_depth[j + 1])) {
                    vector<Point> tmp;
                    tmp.assign(contours_depth[j + 1].begin(), contours_depth[j + 1].end());
                    contours_depth[j + 1].assign(contours_depth[j].begin(), contours_depth[j].end());
                    contours_depth[j].assign(tmp.begin(), tmp.end());
                }
            }
        }

        if (contours_depth.size() > contours_top_view.size())
            for (vector<vector<Point>>::iterator it = contours_depth.begin() + contours_top_view.size();
                 it != contours_depth.end();) {
                it = contours_depth.erase(it);
            }

        vector<vector<Point>> contours_depth_left, contours_depth_right;
        vector<vector<Point>> contours_top_left, contours_top_right;


        for (int i = 0; i < contours_top_view.size(); i++) {
            Rect r1 = boundingRect(contours_top_view[i]);
            if (r1.br().x < top_view.cols / 2)
                contours_top_left.push_back(contours_top_view[i]);
            else
                contours_top_right.push_back(contours_top_view[i]);
        }

        for (int i = 0; i < contours_depth.size(); i++) {
            Rect r1 = boundingRect(contours_depth[i]);
            if (r1.br().x < depth.cols / 2)
                contours_depth_left.push_back(contours_depth[i]);
            else
                contours_depth_right.push_back(contours_depth[i]);
        }

        sort(depth, contours_depth_left, 0);
        sort(depth, contours_depth_right, 0);

        sort(top_view, contours_top_left, 1);
        sort(top_view, contours_top_right, 1);

        for (vector<vector<Point>>::iterator it = contours_top_view.begin(); it != contours_top_view.end();) {
            it = contours_top_view.erase(it);
        }
        for (vector<vector<Point>>::iterator it = contours_depth.begin(); it != contours_depth.end();) {
            it = contours_depth.erase(it);
        }

        contours_top_view.assign(contours_top_left.begin(), contours_top_left.end());
        for (int i = 0; i < contours_top_right.size(); i++) contours_top_view.push_back(contours_top_right[i]);

        contours_depth.assign(contours_depth_left.begin(), contours_depth_left.end());
        for (int i = 0; i < contours_depth_right.size(); i++) contours_depth.push_back(contours_depth_right[i]);
    }
}

void sort(Mat img,vector<vector<Point>> &list,int mode)
{
    if(list.size()>=1) {
        if (mode == 0) //depth    average_depth_max_to_min
        {
            for (int i = 0; i < list.size() - 1; i++)
                for (int j = 0; j < list.size() - i - 1; j++) {
                    if (cal_contours_depth(img, list[j]) < cal_contours_depth(img, list[j + 1])) {
                        vector<Point> tmp;
                        tmp.assign(list[j + 1].begin(), list[j + 1].end());
                        list[j + 1].assign(list[j].begin(), list[j].end());
                        list[j].assign(tmp.begin(), tmp.end());
                    }
                }
        } else        //top_view   center_min_to_max
        {

            for (int i = 0; i < list.size() - 1; i++)
                for (int j = 0; j < list.size() - i - 1; j++) {
                    if (cal_contours_top(img, list[j]) < cal_contours_top(img, list[j + 1])) {
                        vector<Point> tmp;
                        tmp.assign(list[j + 1].begin(), list[j + 1].end());
                        list[j + 1].assign(list[j].begin(), list[j].end());
                        list[j].assign(tmp.begin(), tmp.end());
                    }
                }
        }
    }

}

double cal_contours_depth(Mat depth,vector<Point> contour)
{
    double num=0;
    double sum=0;
    for(int i=0;i<depth.rows;i++)
        for(int j=0;j<depth.cols;j++)
        {
            if(  pointPolygonTest(contour, Point2f(j, i), false) ==1 )
            {
                num+=1.0;
                sum+=depth.at<ushort>(i,j);
            }
        }
    sum=sum/num;
    return sum;
}

double cal_contours_top(Mat top,vector<Point> contour)
{
    Rect rr=boundingRect(contour);
    return double(rr.y+rr.height/2);
}

void initContourList( vector<obstacle> &obstacle_lists,Mat &color,Mat &depth,Mat &topview,vector<vector<Point>> &contours_top_view,vector<vector<Point>> &contours_depth)
{
    //get_depth_contour && delete_invalid_contours
    //vector<vector<Point>> contours_depth;
    vector<Vec4i> hierarchy;
    // depth=stressDepth2(depth);
    Mat depth_tp;
    depth.convertTo(depth_tp,CV_8UC1,255.0/65535.0);

    Mat tpp=convertTo3Channels(depth_tp);
    findContours(depth_tp, contours_depth, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    imshow("d",depth_tp);
//   // waitKey(0);
//    imshow("t",topview);
//    waitKey(0);
    process_Contours(topview,depth, contours_top_view, contours_depth);

//    Mat topp=convertTo3Channels(topview);
//    for(int i=0;i<contours_top_view.size();i++)
//    {
//        Scalar color(rand()%255,rand()%255,rand()%255);
//        cv::drawContours(topp,contours_top_view,i,color,4);
//        cv::drawContours(tpp,contours_depth,i,color,4);
//    }
//    imshow("top",topp);
//    imshow("depth",tpp);
//    waitKey(0);


    int ii=0;
    for( vector<vector<Point>>::iterator it=contours_depth.begin();it!=contours_depth.end();)
    {
        vector<Point> border;
        Rect tpr=boundingRect((*it));
        //      int id=-1;
//        for(int i=0;i<contours_top_view.size();i++)
//        {
//            Rect tprr=boundingRect(contours_top_view[i]);
////                int thres=tpr.width*0.2;
////                if( (tprr.x<tpr.x+thres && tprr.x>tpr.x-thres) && (tprr.x+tprr.width<tpr.x+tpr.width+thres && tprr.x+tprr.width>tpr.x+tpr.width-thres)   )
////                {
////                    //  drawContours(top_tpp,contours,i,Scalar(rand()%255,rand()%255,rand()%255),3);
////                    id=i;
////                }
//            int thres=tprr.width*0.2;
//            if( (tpr.x<tprr.x+thres && tpr.x>tprr.x-thres) && (tpr.x+tpr.width<tprr.x+tprr.width+thres && tpr.x+tpr.width>tprr.x+tprr.width-thres)   )
//            {
//                //  drawContours(top_tpp,contours,i,Scalar(rand()%255,rand()%255,rand()%255),3);
//                id=i;
//            }
//        }
//        if(id!=-1)
//        {
        //  Mat tp_m=color(Range(tpr.y, tpr.y+tpr.height), Range(tpr.x, tpr.x+tpr.width));
        Mat tp_m=getRange2(color,(*it));
//        imshow("tp",tp_m);
//        waitKey(0);
        obstacle tp_o(contours_top_view[ii], contours_depth[ii],   tpr,tp_m);
        obstacle_lists.push_back(tp_o);
        //  drawContours(tpp,contours_tp,it-contours_tp.begin(),Scalar(rand()%255,rand()%255,rand()%255),3);

//        }
        ii++;
        it++;
    }

//    tpp=convertTo3Channels(depth);
//    for(int i=0;i<contours_depth.size();i++)
//    {
//        drawContours(tpp,contours_depth,i,Scalar(rand()%255,rand()%255,rand()%255),3);
//    }
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

void preprocessImage(Mat &image)
{

//    Mat element1 = getStructuringElement(MORPH_RECT, Size(2,2)); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
//    //膨胀操作
//    dilate(image, image, element1);
//    cv::namedWindow("image_tp", WINDOW_NORMAL);
//    cv::imshow("image_tp",image);
//    cv::waitKey(0);

//    vector<vector<Point>> contours;
//    vector<Vec4i> hierarchy;
//    findContours(image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    for(int i=0;i<contours.size();i++)
//    {
//        if(contourArea(contours[i])<=5) {
//            cv::Rect r0 = cv::boundingRect(cv::Mat(contours[i]));
//            //  rectangle(image,r0.tl(),r0.br(),cv::Scalar(100,100,100),5,8);
////            cv::namedWindow("tpp",WINDOW_NORMAL);
////            imshow("tpp",image);
////            cv::waitKey(0);
//            for (int r = r0.y; r < r0.y + r0.height; r++)
//                for (int c = r0.x; c < r0.x + r0.width; c++)
//                    image.at<uchar>(r, c) = 0;
//        }
//    }

//    cv::namedWindow("image_tp", WINDOW_NORMAL);
//    cv::imshow("image_tp",image);
//    cv::waitKey(0);

    Mat element1 = getStructuringElement(MORPH_RECT, Size(3,3));
    Mat top_view1;
    dilate(image, image, element1);

    vector<vector<Point>> contours1,contours2;
    vector<Vec4i> hierarchy1,hierarchy2;
    findContours(image, contours1, hierarchy1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for(int k=0;k<contours1.size();k++)
    {
        if(contourArea((contours1[k]))<60)
        {
            for(int i=0;i<image.rows;i++)
                for(int j=0;j<image.cols;j++)
                    if(  pointPolygonTest(contours1[k], Point2f(j, i), false) !=-1)
                        image.at<uchar>(i,j)=0;
        }

    }
    element1 = getStructuringElement(MORPH_RECT, Size(8,8));
    dilate(image, image, element1);

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

            if(image.at<uint16_t>(i,j)>45000)result.at<uint16_t >(i,j)=0;
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

Mat getRange2(Mat &img,vector<Point> contour)
{
    Rect rr=boundingRect(contour);
    Mat result=Mat::zeros(rr.height+1,rr.width+1,CV_8UC3);
    for(int i=rr.y;i<=rr.y+rr.height;i++)
        for(int j=rr.x;j<=rr.x+rr.width;j++)
            for(int k=0;k<3;k++)
            {
                if(  pointPolygonTest(contour, Point2f(j, i), false) ==1 )
                    result.at<Vec3b>(i-rr.y,j-rr.x)[k]=img.at<Vec3b>(i,j)[k];
            }
    return result;
}

vector<double> normalize_list(vector<double> list)
{
    vector<double> nor;
    double min=10000,max=0;
    for(int i=0;i<list.size();i++)
    {
        if(list[i]>max)max=list[i];
        if(list[i]<min)min=list[i];
    }

    if(max==min)
        for(int i=0;i<list.size();i++)
        {
            nor.push_back(0);
        }

    for(int i=0;i<list.size();i++)
    {
        nor.push_back((list[i]-min)/(max-min));
    }
    return nor;
}




/*
    base_test_cmd.linear.x = 0;
    base_test_cmd.linear.y= 0;
    base_test_cmd.angular.x=0;
    base_test_cmd.angular.y=0;
    base_test_cmd.angular.z=0;
 pub_move.publish(base_test_cmd);
*/