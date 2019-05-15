#!/usr/bin/env bash
# //////////////////////////////////////////////////////////////////////////////
# display usage
usage_msg="\
Usage: $(basename $0)
options:
  * optional arguments are enclosed in square brackets

  [--robot <robot>]
      run builder on remote host
  [--ssh < host@IP >]
      run builder on remote host, of form 'host@IP'
  [--path <deploy-path>]
      path to deploy repo src directory on host
  [--show]
      shows ssh hosts configured with script
  [--preview]
      preview docker build command; docker build is not called
  [--all]
      go through all hosts and perform the build
  
  Runs deploy builder, locally or remotely.

For more help, please see the README.md or wiki."

# //////////////////////////////////////////////////////////////////////////////
# print nicely

# colors
DEFCOL="\e[39m"
GREEN="\e[32m"
BLUE="\e[34m"
RED="\e[31m"
YELLOW="\e[33m"
CYAN="\e[36m"

# formatters
divider_large() { printf "=======================================================================================================\n"; }
title() { echo -e "\t\t\t\t\t $GREEN == $1 == $DEFCOL \n\n"; }
display_text() { echo -e "$GREEN $1 $DEFCOL\n"; }
warning() { echo -e "$YELLOW$1 $DEFCOL\n"; }
print_error() { echo -e "$RED$1 $DEFCOL\n"; }
exit_success() { exit 0; }
exit_failure() { exit 1; }
newline() { echo -e "\n"; }
validate() { if [ $? -ne 0 ]; then print_error "$1"; exit_failure; fi; }

# //////////////////////////////////////////////////////////////////////////////
# ssh hosts
. "$(dirname $0)/builder-hosts"
validate "load builder-hosts failed"

# //////////////////////////////////////////////////////////////////////////////
# arguments
preview=false
all=false
while [[ $# -gt 0 ]]
do
  key="${1}"
  case ${key} in
  --robot)
    robot_host="${2}"
    shift # past argument
    shift # past value
    ;;
  --ssh)
    ssh_conn="${2}"
    shift # past argument
    shift # past value
    ;;
  --path)
    path="${2}"
    shift # past argument
    shift # past value
    ;;
  --deploy)
    args="${2}"
    shift # past argument
    shift # past value
    ;;
  --all)
    all=true
    shift # past argument
    ;;
  --preview)
    preview=true
    shift # past argument
    ;;
  --show)
    show=true
    shift # past argument
    ;;
  help)
    help=true
    shift # past argument
    ;;
  --help)
    help=true
    shift # past argument
    ;;
  *)      # unknown option
    print_error "unkown argument '$1'"
    echo -e "$YELLOW$usage_msg$DEFCOL\n";
    exit_failure
    ;;
  esac
done

# prints
divider_large
title "Deploy Builder"

# //////////////////////////////////////////////////////////////////////////////
# deploy builder helper

# go through ssh array, display.
show_config() {
  printf "$GREEN%10s %10s $DEFCOL\n" "user" "host"
  echo -e "   ------------------------"
  for i in "${arr[@]}"; do
    printf "$CYAN%10s %10s $DEFCO\n" $(echo "$i" | cut -d'@' -f1) $(echo "$i" | cut -d'@' -f2)
  done
  display_text "\n   path: $path \n"
}

# setup the ssh connection, if given as variable
robot_builder() {
  # set the host arg
  local host="$1"
  # find the matching host in arr of users@hosts
  for i in "${arr[@]}"; do
    if [ "$host" == "$(echo "$i" | cut -d'@' -f2)" ]; then
      ssh_conn="$i"
      return
    fi
  done
  # nothing found, throw error
  print_error "No matching ssh connection. Please see [--show] ssh configuration set."
  exit_failure  
}

# perform the builder
eval_builder() {
  # perform the builder
  display_text "$dbuilder"
  if [ $preview == false ]; then eval $dbuilder; fi
  validate "deploy builder failed."
}

setup_dbuilder() {
  # setup the builder command
  dbuilder="cd $path && ./builder $args"
  # setup the builder ssh connection
  [[ "$ssh_conn" ]] && { dbuilder="ssh $ssh_conn '$dbuilder'"; }
}

# //////////////////////////////////////////////////////////////////////////////
# deploy builder

[[ "$robot_host" ]] && {  robot_builder "$robot_host"; }
[[ "$show" ]] && {  show_config; exit_success; }
[[ "$help" ]] && { echo -e "$YELLOW$usage_msg$DEFCOL\n"; exit_failure; }

# multiple hosts evaluation
if [ $all == true ]; then
  for i in "${arr[@]}"; do
    # setup the robot builder
    robot_builder "$(echo "$i" | cut -d'@' -f2)"
    display_text "== Builder on $(echo "$i" | cut -d'@' -f2) =="
    setup_dbuilder
    eval_builder
    newline
  done
# single host evaluation
else
  setup_dbuilder
  eval_builder
fi;

# cleanup & exit
exit_success

