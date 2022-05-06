bag = rosbag('trial.bag');
sub = select(bag,'Topic','/device_0/sensor_1/Color_0/image/data');
msg = readMessages(sub,1)
data = msg{1,1}.readImage;

