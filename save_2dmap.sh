
cmds=(  "python3 ./src/navigation/robot_navigation2/map_trans_tool/pcd2pgm_simplify.py ")
# ros2 run nav2_map_server map_saver_cli -f test

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2 
done



