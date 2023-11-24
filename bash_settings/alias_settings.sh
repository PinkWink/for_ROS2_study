# ALIAS settings
ID=13

alias killgazebo="killall gzserver gzclient"

alias sz="source ~/.zshrc; echo \"Zsh is reloaded!\""
alias ros_domain="export ROS_DOMAIN_ID=\$ID; echo \"ROS_DOMAINID is set to \$ID !\""
alias humble="source /opt/ros/humble/setup.zsh; ros_domain; echo \"ROS2 Humble is activated\"; tmp2"

ws_setting()
{
	humble
	source ~/$1/install/local_setup.zsh
	echo "$1 workspace is activated."
	tmp2
}

alias  my_mobile="ws_setting \"my_mobile\""
alias  urdf_study="ws_setting \"urdf_study\""