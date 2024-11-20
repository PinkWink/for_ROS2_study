# argcomplete for ros2 & colcon
alias tmp1="eval \"\$(register-python-argcomplete3 ros2)\""
alias tmp2="tmp1; eval \"\$(register-python-argcomplete3 colcon)\""

# ALIAS settings
ID=13

alias killgazebo="killall gzserver gzclient"

alias sz="source ~/.zshrc; echo \"Zshrc is reloaded!\""
alias sb="source ~/.bashrc; echo \"Bashrc is reloaded!\""

alias ros_domain="export ROS_DOMAIN_ID=\$ID; echo \"ROS_DOMAINID is set to \$ID !\""
alias humble="source /opt/ros/humble/setup.bash; ros_domain; echo \"ROS2 Humble is activated\""

ws_setting()
{
	humble
	source ~/$1/install/local_setup.bash
	echo "$1 workspace is activated."
	tmp2
}

get_status()
{
	if [ -z $ROS_DOMAIN_ID ]; then
		echo "ROS_DOMAIN_ID : 0"
	else
		echo "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
	fi

	if [ -z $ROS_LOCALHOST_ONLY ]; then
		echo "ROS_LOCALHOST_ONLY : 0"
	else
		echo "ROS_LOCALHOST_ONLY : $ROS_LOCALHOST_ONLY"
	fi
}

alias  ros2_study="ws_setting \"ros2_study\""

