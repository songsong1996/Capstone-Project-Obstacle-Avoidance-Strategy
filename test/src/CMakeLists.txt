

set(OpenCV_DIR "/home/songsong/Documents/Softwares/opencv-4.0.1/build")

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )



#ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
#TARGET_LINK_LIBRARIES( generate_pointcloud ${OpenCV_LIBS})

ADD_EXECUTABLE( tp main2.cpp )
TARGET_LINK_LIBRARIES( tp ${OpenCV_LIBS})
