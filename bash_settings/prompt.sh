GREEN='\[\e[1;32m\]'
BLUE='\[\e[1;34m\]' 
NC='\[\e[0m\]'

PS1="${BLUE}(ID: \${ROS_DOMAIN_ID:-0})${GREEN}\u${NC}:${BLUE}\w${NC}\$ "
