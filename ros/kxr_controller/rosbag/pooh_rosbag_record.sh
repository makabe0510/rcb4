# rosbag file is generated in directry where this script is executed.
# ./pooh_rosbag_record.sh [bag-name]
# e.g.
# ./pooh_rosbag_record.sh log-take1
rosbag record -o $1 /joint_states /eye_brow/joint_states
