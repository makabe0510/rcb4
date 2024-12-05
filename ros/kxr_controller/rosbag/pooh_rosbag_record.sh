# rosbag file is generated in directry where this script is executed.
# ./pooh_rosbag_record.sh [bag-name]
# e.g.
# $ roscd kxr_controller/rosbag
# $ rosrun kxr_controller pooh_rosbag_record.sh log-take1
# take1_20XX-XX-XX-XX-XX-XX.bag is genereted in kxr_controller/rosbag
rosbag record -o $1 /joint_states /eye_brow/joint_states
