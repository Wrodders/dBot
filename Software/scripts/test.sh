gst-launch-1.0 libcamerasrc !  \
capsfilter caps=video/x-raw,width=640,height=360,format=NV12 ! \
v4l2convert ! v4l2h264enc extra-controls="controls,repeat_sequence_header=1" ! \
'video/x-h264,level=(string)4.1' ! \
h264parse ! \
rtph264pay ! \
udpsink host=192.168.0.32 port=5001 sync=false