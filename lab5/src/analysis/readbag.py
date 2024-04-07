import rosbag
bag = rosbag.Bag('/home/rohit/.ros/testimu.bag')
for topic,msg, t in bag.read_messages(topics=['imu']):
    print(msg)
    #print(t)
bag.close()