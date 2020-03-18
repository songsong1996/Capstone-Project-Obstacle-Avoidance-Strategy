//
// Created by songsong on 19-5-12.
//


#include"readImages.h"
#include <unistd.h>
#include"contours_processor.h"

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

using namespace std;
////using namespace cv;

int main( int argc, char** argv )
{

    string filepath1="../data/pcd/top_view/";
    string filepath2="../data//pcd/color/";
    string filepath3="../data/pcd/depth2/";



    vector<string> file_list1,file_list2,file_list3;
    int tp;
    tp=readFileList(filepath1,file_list1);
    tp=readFileList(filepath2,file_list2);
    tp=readFileList(filepath3,file_list3);
//    for(int i=0;i<file_list.size();i++)
//    {
//        cout<<file_list.at(i)<<endl;
//    }

    //match(vector<string> color_list,vector<string> depth_list,vector<string> top_view);
match new_folder(file_list2,file_list3,file_list1);

//for(int i=0;i<file_list.size()-1;i++)
//{
//    Mat previous,present;
//    previous= imread(file_list.at(i),IMREAD_UNCHANGED );
//    present=imread(file_list.at(i+1),IMREAD_UNCHANGED );
//    new_folder.match_contours(previous,  present) ;
//
//}


cout<<"finish"<<endl;

pause();
    return 0;
}


