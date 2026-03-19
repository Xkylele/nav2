
cmds=(  "ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=map.bt -p save_path:=/home/lele/ros2_humble_main")




for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2 
done

