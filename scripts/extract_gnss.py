import rosbag
import pandas as pd
from sensor_msgs.msg import NavSatFix
import gmplot


# def extract_lla(bagfile, topic_name):
#     bag = rosbag.Bag(bagfile, 'r')
#     lla_data = []

#     for topic, msg, t in bag.read_messages(topics=[topic_name]):
#         # print(f"Topic: {topic}, Time: {t}")
#         lla_data.append({'time': t.to_sec(), 'latitude': msg.latitude, 'longitude': msg.longitude, 'altitude': msg.altitude, 'status': msg.status.status, 'service': msg.status.service})

#     bag.close()
#     return pd.DataFrame(lla_data)

# bagfile = '02_mcap.bag'
# topic_name = '/device/gnss_base/fix'
# lla_df = extract_lla(bagfile, topic_name)

# if not lla_df.empty:
#     gmap = gmplot.GoogleMapPlotter(lla_df['latitude'].iloc[0], lla_df['longitude'].iloc[0], 20)
#     gmap.scatter(lla_df['latitude'], lla_df['longitude'], '#FF0000', size=0.5, marker=False)
#     gmap.plot(lla_df['latitude'], lla_df['longitude'], 'cornflowerblue', edge_width=0.2)
# else:
#     print("No data available from ROS bag to plot.")


# txt_path = "/home/jfeng/.ros/state_data.txt"
# new_lla = []
# with open(txt_path, 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#         line = line.strip().split(',')
#         new_lla.append([float(line[-3]), float(line[-2]), float(line[-1])])

# if new_lla:
#     latitudes, longitudes, altitudes = zip(*new_lla)
#     gmap.scatter(latitudes, longitudes, '#00FF00', size=0.5, marker=False)
#     gmap.plot(latitudes, longitudes, 'cornflowerblue', edge_width=0.2)
#     print("Map has been generated.")
# else:
#     print("No data available to plot.")

# gmap.draw('combined_map2.html')

def extract_lla(bagfile, topic_name):
    bag = rosbag.Bag(bagfile, 'r')
    lla_data = []

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        print(f"status: {msg.status.status}, Time: {t}")
        lla_data.append({'status': msg.status.status, 'service': msg.status.service})

    bag.close()
    return pd.DataFrame(lla_data)

bagfile = '/home/jfeng/Data/RDX_Stage01/PublicDatasets/ROS_Data/HGSC/2/new_sensors_03.bag'
topic_name = '/device/gnss_base/fix'
lla_df = extract_lla(bagfile, topic_name)

# save llas to txt file
txt_path = "./gnss_data.csv"
lla_df.to_csv(txt_path, index=False)

# plot the histogram of status
status_hist = lla_df['status'].value_counts()
print(status_hist)