//
// Created by songsong on 19-6-6.
//

int match::find_same(obstacle &obs,vector<obstacle> &obstacle_lists,int &is_exist)
{

    is_exist=0;
    if(obstacle_lists.size()==0)return 0;
    int num=0;
    int hash_min=100;
    imshow("color1",obs.color);
    waitKey(0);

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
        imshow("color_num_-1",obstacle_lists[num].color);
        waitKey(0);
        double d2 =(double) obstacle_lists[num].depth_contour.width / (double) obstacle_lists[num].depth_contour.height;
        cout<<"d2="<<d2<<"  d1="<<d0<<endl;

        d0=obs.depth_contour.width*obs.depth_contour.height;
        d2=obstacle_lists[num].depth_contour.width*obstacle_lists[num].depth_contour.height;

        if (d2>d0*1.2 || d2<d0*0.8)
            num=-1;
    }
    if(num==-1) //hash==hash_min
    {

        double r0=0,g0=0,b0=0;
        double num0=0;
        for(int rr=0;rr<obs.color.rows;rr++)
            for(int cc=0;cc<obs.color.cols;cc++)
            {
                if(  obs.color.at<Vec3b>(rr,cc)!=Vec3b(0,0,0) )
                {
                    num0+=1.0;
                    b0+=(double)obs.color.at<Vec3b>(rr,cc)[0];
                    g0+=(double)obs.color.at<Vec3b>(rr,cc)[1];
                    r0+=(double)obs.color.at<Vec3b>(rr,cc)[2];
                }
            }

        b0/=num0;
        g0/=num0;
        r0/=num0;


        double dist_min;
        cout<<"obstacle_num_num="<<obstacle_lists.size()<<endl;
        for(int i=0;i<obstacle_lists.size();i++)
        {
            double r=0,g=0,b=0;
            double num1=0;
            for(int rr=0;rr<obstacle_lists[i].color.rows;rr++)
                for(int cc=0;cc<obstacle_lists[i].color.cols;cc++)
                {
                    if(  obstacle_lists[i].color.at<Vec3b>(rr,cc)!=Vec3b(0,0,0) )
                    {
                        num1+=1.0;
                        b+=(double)obstacle_lists[i].color.at<Vec3b>(rr,cc)[0];
                        g+=(double)obstacle_lists[i].color.at<Vec3b>(rr,cc)[1];
                        r+=(double)obstacle_lists[i].color.at<Vec3b>(rr,cc)[2];
                    }
                }

            b/=num1;
            g/=num1;
            r/=num1;

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
        imshow("color_match_dist",obstacle_lists[num].color);
        waitKey(0);
    }

//    if (hash_min<30)
//    {
//        cout<<"num="<<num<<endl;
//        cout<<endl;

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
    }




//        is_exist=1;
//    }
    return num;
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

    for(int i=0;i<file_list.size()-1;i++)
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
        match_contours2(previous, color_previous,depth_previous,present,color_present,depth_present);
    }
}