#!/bin/bash 
GREEN='\033[0;32m'
LGREEN='\033[1;32m'

ORANGE='\033[0;33m'

RED='\033[0;31m'
LRED='\033[1;31m'

BLUE='\033[0;34m'
LBLUE='\033[1;34m'

PURPLE='\033[0;35m'
LPURPLE='\033[1;35m'

CYAN='\033[0;36m'
LCYAN='\033[1;36m'

NC='\033[0m'

cat $(pwd)/LOGO.txt

# a="$GREEN Deploy bash script $ENDC"
# echo "$a"

python3 ./scripts/deploy.py $(pwd)



