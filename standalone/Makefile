blob_tracker: simple_blob_tracker4.cpp
	g++ -std=c++11 -g -Wall -O2 -o simple_blob_tracker4.out simple_blob_tracker4.cpp -fpermissive -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_calib3d

capture: capture.cpp 
	g++ -g -Wall -O2 -o capture.out capture.cpp -lopencv_core -lopencv_highgui -lopencv_videoio

#obj_det: obj_det_track3.cpp 
#	g++ -I -Wall obj_det_track3.cpp -o obj_det_track3.out -lopencv_core -lopencv_video -lopencv_imgproc -lopencv_highgui -lopencv_videoio

all: blob_tracker capture

clean:
	rm simple_blob_tracker4.out
	rm capture.out
	rm obj_det_track3.out
