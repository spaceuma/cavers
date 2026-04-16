# Message header: timestamp should be acquisition time of image, frame_id should be optical frame of camera 
std_msgs/Header header

# Image height, that is, number of rows
uint32 height

# Image width, that is, number of columns
uint32 width

# Temperature matrix data, size is (height * width)
float32[] data