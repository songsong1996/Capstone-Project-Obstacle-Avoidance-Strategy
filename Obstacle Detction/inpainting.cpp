//
// Created by songsong on 19-5-30.
//

#include "inpainting.h"



Mat preprocessor::Total_Variation(Mat img,int iter)
{
    int i, j;
    int nx = img.rows, ny = img.cols;
    double ep2 = epsilon * epsilon;

    vector<Mat> channels,channels_new;
    split(img,channels);
    for(int kk=0;kk<3;kk++)
    {
        Mat I_t=Mat::zeros(nx,ny,CV_8UC1);
        Mat I_tmp=Mat::zeros(nx,ny,CV_8UC1);

        channels[kk].copyTo(I_t);
        channels[kk].copyTo(I_tmp);


        for (int t = 0; t < iter; t++)
        {
            for (i = 0; i < nx; i++)
            {
                for (j = 0; j < ny; j++)
                {
                    int iUp = i - 1, iDown = i + 1;
                    int jLeft = j - 1, jRight = j + 1;    // 边界处理
                    if (0 == i) iUp = i; if (nx - 1 == i) iDown = i;
                    if (0 == j) jLeft = j; if (ny - 1 == j) jRight = j;

                    double tmp_x = (I_t.at<uchar>(i,jRight) - I_t.at<uchar>(i,jLeft)) / 2.0;
                    double tmp_y = (I_t.at<uchar>(iDown,j) - I_t.at<uchar>(iUp,j)) / 2.0;
                    double tmp_xx = I_t.at<uchar>(i,jRight) + I_t.at<uchar>(i,jLeft) - 2 * I_t.at<uchar>(i,j);
                    double tmp_yy = I_t.at<uchar>(iDown,j) + I_t.at<uchar>(iUp,j) - 2 * I_t.at<uchar>(i,j);
                    double tmp_xy = (I_t.at<uchar>(iDown,jRight) + I_t.at<uchar>(iUp,jLeft) - I_t.at<uchar>(iUp,jRight) - I_t.at<uchar>(iDown,jLeft)) / 4.0;
                    double tmp_num = tmp_yy * (tmp_x * tmp_x + ep2) + tmp_xx * (tmp_y * tmp_y + ep2) - 2 * tmp_x * tmp_y * tmp_xy;
                    double tmp_den = pow(tmp_x * tmp_x + tmp_y * tmp_y + ep2, 1.5);

                    I_tmp.at<uchar>(i,j) += dt*(tmp_num / tmp_den + lambda*(double)(channels[kk].at<uchar>(i,j) - I_t.at<uchar>(i,j)));
                }
            }  // 一次迭代

            for (i = 0; i < nx; i++)
                for (j = 0; j < ny; j++)
                {
                    I_t.at<uchar>(i,j) = I_tmp.at<uchar>(i,j);
                }
        } // 迭代结束

        Mat result=Mat::zeros(nx,ny,CV_8UC1);
        // 给图像赋值
        for (i = 0; i < nx; i++)
            for (j = 0; j < ny; j++)
            {
                double tmp = I_t.at<uchar>(i,j);
                double tptmp=tmp>255? 255:tmp;
                double tpptmp=tptmp>0? tptmp:0;
                //   tmp = max(0, min(tmp, 255));
                result.at<uchar>(i,j) = (float)tpptmp;
            }
        channels_new.push_back(result);
        imshow("tpp",result);
        waitKey(0);

    }

    Mat rr;
    merge(channels_new,rr);
    imshow("tpp",rr);
    waitKey(0);
    return rr;
}


Mat Total_Variation2(Mat img,int iter)
{
    double  dt=0.25, epsilon=1.0,lambda=0;
    int i, j;
    int nx = img.rows, ny = img.cols;
    double ep2 = epsilon * epsilon;

    Mat I_t=Mat::zeros(nx,ny,CV_8UC1);
    Mat I_tmp=Mat::zeros(nx,ny,CV_8UC1);

    img.copyTo(I_t);
    img.copyTo(I_tmp);


    for (int t = 0; t < iter; t++)
    {
        for (i = 0; i < nx; i++)
        {
            for (j = 0; j < ny; j++)
            {
                int iUp = i - 1, iDown = i + 1;
                int jLeft = j - 1, jRight = j + 1;    // 边界处理
                if (0 == i) iUp = i; if (nx - 1 == i) iDown = i;
                if (0 == j) jLeft = j; if (ny - 1 == j) jRight = j;

                double tmp_x = (I_t.at<uchar>(i,jRight) - I_t.at<uchar>(i,jLeft)) / 2.0;
                double tmp_y = (I_t.at<uchar>(iDown,j) - I_t.at<uchar>(iUp,j)) / 2.0;
                double tmp_xx = I_t.at<uchar>(i,jRight) + I_t.at<uchar>(i,jLeft) - 2 * I_t.at<uchar>(i,j);
                double tmp_yy = I_t.at<uchar>(iDown,j) + I_t.at<uchar>(iUp,j) - 2 * I_t.at<uchar>(i,j);
                double tmp_xy = (I_t.at<uchar>(iDown,jRight) + I_t.at<uchar>(iUp,jLeft) - I_t.at<uchar>(iUp,jRight) - I_t.at<uchar>(iDown,jLeft)) / 4.0;
                double tmp_num = tmp_yy * (tmp_x * tmp_x + ep2) + tmp_xx * (tmp_y * tmp_y + ep2) - 2 * tmp_x * tmp_y * tmp_xy;
                double tmp_den = pow(tmp_x * tmp_x + tmp_y * tmp_y + ep2, 1.5);

                I_tmp.at<uchar>(i,j) += dt*(tmp_num / tmp_den + lambda*(double)(img.at<uchar>(i,j) - I_t.at<uchar>(i,j)));
            }
        }  // 一次迭代

        for (i = 0; i < nx; i++)
            for (j = 0; j < ny; j++)
            {
                I_t.at<uchar>(i,j) = I_tmp.at<uchar>(i,j);
            }
    } // 迭代结束

    Mat result=Mat::zeros(nx,ny,CV_8UC1);
    // 给图像赋值
    for (i = 0; i < nx; i++)
        for (j = 0; j < ny; j++)
        {
            double tmp = I_t.at<uchar>(i,j);
            double tptmp=tmp>255? 255:tmp;
            double tpptmp=tptmp>0? tptmp:0;
            //   tmp = max(0, min(tmp, 255));
            result.at<uchar>(i,j) = (float)tpptmp;
        }
//        channels_new.push_back(result);
//        imshow("tpp",result);
//        waitKey(0);



//    Mat rr;
//    merge(channels_new,rr);
    imshow("tpp",result);
    waitKey(0);
    return result;
}


preprocessor::preprocessor(Mat col,Mat dep,int iter)
{
    col.copyTo(color);
    dep.copyTo(depth);
    color.copyTo(tmp_color);
    depth.copyTo(tmp_depth);
    structure_component=Total_Variation(color,1);
    beta=3;
    iter_num=iter;
    h1=2.24;
    h2=0.22;
    h3=0.08;
    dt=0.25, epsilon=1.0,lambda=0;

}
vector<Point> preprocessor::circle_list(int i0,int j0,int radius)
{
    vector<Point> list;
    int d=floor(radius/sqrt(2));
    for(int i=1;i<=radius;i++)
    {
        list.push_back( Point(i0+i,j0+i));
        list.push_back( Point(i0+i,j0-i));
        list.push_back( Point(i0-i,j0+i));
        list.push_back( Point(i0-i,j0-i));
    }
    for(int i=1;i<radius;i++)
    {
        list.push_back( Point(i0+i,j0));
        list.push_back( Point(i0,j0+i));
        list.push_back( Point(i0-i,j0));
        list.push_back( Point(i0,j0-i));
    }
    list.push_back( Point(i0,j0));
    return list;
}


double preprocessor::compute_dp(Mat depth,int i0,int j0)
{
    //find_depth_proper
    vector<Point> lists=circle_list(i0,j0,radius);

    vector<double> w_lists;
    double d_max=0;
    for(int i=0;i<lists.size();i++)
    {
        if(depth.at<ushort>(lists[i].x,lists[i].y)!=0) {
            if (depth.at<ushort>(lists[i].x, lists[i].y) > d_max)
                d_max = depth.at<ushort>(lists[i].x, lists[i].y);
        }
    }

    double w_sum=0;
    for(int i=0;i<lists.size();i++)
    {
        if(depth.at<ushort>(lists[i].x,lists[i].y)!=0)
        {
            double w1=-compute_w1(i0,j0,lists[i].x,lists[i].y)/(2.0*h1*h1);
            double w2=-compute_w2(depth,i0,j0,lists[i].x,lists[i].y,d_max)/(2.0*h2*h2);
            double w3=-compute_w3(i0,j0,lists[i].x,lists[i].y)/(2.0*h3*h3);
            double w=exp(w1+w2+w3);
            w_lists.push_back(w);
            w_sum+=w;
        }
    }

    for(int i=0;i<w_lists.size();i++)
    {
        w_lists[i]=w_lists[i]/w_sum;
    }


    double dp=0;
    int j=0;
    for(int i=0;i<lists.size();i++)
    {
        if (depth.at<ushort>(lists[i].x, lists[i].y) != 0)
        {

            dp+=w_lists[j]*(depth.at<ushort>(lists[i].x,lists[i].y)*1.0
                            +(double)(i0-lists[i].x)*(depth.at<ushort>(lists[i].x+1, lists[i].y)-depth.at<ushort>(lists[i].x-1, lists[i].y))
                            +  (double)(j0-lists[i].y)*(depth.at<ushort>(lists[i].x, lists[i].y+1)-depth.at<ushort>(lists[i].x, lists[i].y-1)));
            j++;
        }
    }
    return dp;
}



Mat preprocessor::Structure_Fusion()
{
    Mat tp;
    depth.copyTo(tp);
    for(int k=0;k<iter_num;k++)
    {
        Mat tpp;
        tp.copyTo(tpp);
        for(int i=radius;i<tp.rows-radius;i++)
            for(int j=radius;j<tp.cols-radius;j++)
            {
                tpp.at<ushort>(i,j)=compute_dp(tp,i,j);
            }
        tpp.copyTo(tp);
    }
    return tp;
}






double preprocessor::compute_w1(int i0,int j0,int i,int j)
{
    return  pow(i-i0,2.0)+pow(j-j0,2.0);
}

double preprocessor::compute_w2(Mat depth,int i0,int j0,int i,int j,double d_max)
{
    double d=depth.at<char>(i,j)*1.0;
    return pow( 1- d/d_max ,2.0);
}

double preprocessor::compute_w3(int i0, int j0, int i, int j)
{
    int l=beta/2;
    double sum=0;
    for (int ii=-l;ii<=l;ii++)
        for(int jj=-l;jj<=l;jj++)
        {

            int i1,j1,i2,j2;
            if(ii+i0<0)i1=0;
            else if(ii+i0>=color.rows-1)i1=color.rows-1;
            else i1=ii+i0;
            if(ii+i<0)i2=0;
            else if(ii+i>=color.rows-1)i2=color.rows-1;
            else i2=ii+i;

            if(jj+j0<0)j1=0;
            else if(jj+j0>=color.cols-1)j1=color.cols-1;
            else j1=jj+j0;
            if(jj+j<0)j2=0;
            else if(jj+j>=color.cols-1)j2=color.cols-1;
            else j2=jj+j;

            for (int k=0;k<3;k++)
                sum+=pow( structure_component.at< Vec3b>(i1,j1)[k]-structure_component.at< Vec3b>(i2,j2)[k]  ,2.0);
        }

    return sum;
}