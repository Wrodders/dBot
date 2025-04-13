cmake     -D CMAKE_BUILD_TYPE=RELEASE     
-D CMAKE_INSTALL_PREFIX=/usr/local     
-D BUILD_LIST=core,imgproc,videoio,imgcodecs,video     
-D BUILD_opencv_apps=OFF    
-D BUILD_EXAMPLES=OFF    
-D BUILD_TESTS=OFF    
-D BUILD_PERF_TESTS=OFF     
-D BUILD_JAVA=OFF     
-D BUILD_opencv_python2=OFF     
-D BUILD_opencv_python3=OFF     
-D WITH_GTK=OFF    
-D WITH_QT=OFF     
-D WITH_V4L=ON     
-D WITH_FFMPEG=ON    
-D ENABLE_NEON=ON     
-D ENABLE_VFPV3=ON     
-D OPENCV_ENABLE_NONFREE=OFF     
..
