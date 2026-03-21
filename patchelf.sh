
cmds=(
	"patchelf   --set-rpath "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib"   --force-rpath   /home/lele/Desktop/nav2/install/fast_lio/lib/fast_lio/fastlio_mapping"
	"patchelf   --set-rpath "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib"   --force-rpath   /home/lele/Desktop/nav2/install/icp_registration/lib/icp_registration/icp_registration_node"
	"patchelf   --set-rpath "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib"   --force-rpath   /home/lele/Desktop/nav2/install/terrain_analysis_ext/lib/terrain_analysis_ext/pathNorm"	
	"patchelf   --set-rpath "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib"   --force-rpath   /home/lele/Desktop/nav2/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis"	
)




for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2 
done

