g++ -std=c++17 $(pkg-config --cflags --libs opencv4) $(pkg-config --cflags --libs libzmq cppzmq) -o opencv_rtsp tools/opencv_rtsp.cpp 
