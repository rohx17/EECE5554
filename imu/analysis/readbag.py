import rosbag
bag = rosbag.Bag('/home/rohit/EECE5554/imu/src/data/imu10min.bag')
for topic,msg, t in bag.read_messages(topics=['imu']):
    print(msg)
    #print(t)
bag.close()