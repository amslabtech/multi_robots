import rosbag
import csv
import sys
import os

args = sys.argv
assert len(args) >= 2, "\nyou must specify the bagfile path\n    python3 rosbag_analisys.py BAGFILE [OUTPUT_NAME] "

filename = os.path.normpath(os.path.join(os.getcwd(), args[1]))
print("target:")
print("    ", filename)

bag = rosbag.Bag(filename)

print("save topics:")
expected_topics = list()
for topic_name in bag.get_type_and_topic_info().topics:
    expected_topics.append(topic_name)
    print("    ", topic_name)

output_file = args[2] if len(args) == 3 else "output.csv"
print("output file:")
print("    ", output_file)

messages = dict();
for topic, msg, time in bag.read_messages():
    index = -1;
    for i, name in enumerate(expected_topics):
        if topic == name:
            index = i
    if index == -1:
        continue

    if time.secs not in messages:
        messages[time.secs] = [0] * len(expected_topics)
    messages[time.secs][index] += 1

sorted_messages = sorted(messages.items(), key=lambda x:x[0])
start_time = sorted_messages[0][0]
with open(output_file, 'w') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["sec"] + expected_topics)
    for time, nums in sorted_messages:
        writer.writerow([time - start_time] + nums)

bag.close()
