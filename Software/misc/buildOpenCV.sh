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



        
        # Start the video input process
        # Opens the camera using libcamerasrc, converts the video format to I420, and writes it to the named pipe
        sudo gst-launch-1.0 libcamerasrc ! videoconvert ! videorate ! \
            videoflip method=$ROT ! \
            video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! \
            filesink location=$VIDEO_IN &

        # Start the video output process to the selected port
        # Reads the video from the named pipe, encodes it using x264, and sends it over UDP rtp
        sudo gst-launch-1.0 filesrc location=$VIDEO_OUT blocksize=460800 do-timestamp=true ! \
            videoparse format=i420 width=640 height=480 framerate=30/1 ! \
            x264enc tune=zerolatency speed-preset=ultrafast bitrate=5000 key-int-max=30 ! \
            h264parse ! \
            'video/x-h264,profile=baseline,level=(string)4.1' ! \
            rtph264pay config-interval=-1 pt=96 ! \
            udpsink host=192.168.0.32 port=$PORT sync=false async=false &


        # Start vision navigation process 
        # Reads the video from the named pipe, processes it using the vision program, and writes the output to the other named pipe
        ./build/vision