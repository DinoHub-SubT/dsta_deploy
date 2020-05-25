#!/usr/bin/env bash
# //////////////////////////////////////////////////////////////////////////////
# display usage
usage_msg="\
Usage: $(basename $0)

Options:
  --install
      install the source script in the devel setup.bash
  --uninstall
      uninstall the source script in the devel setup.bash

  Installs or uninstalls roslaunch params in the devel setup.bash

For more help, please see the README.md or wiki."

### helpers
# colors
DEFCOL="\e[39m"
YELLOW="\e[33m"
RED="\e[31m"
exit_success() { exit 0; }
exit_failure() { exit 1; }
print_error() { echo -e "$RED$1 $DEFCOL\n"; }
validate() { if [ $? -ne 0 ]; then print_error "$1"; exit_failure; fi; }

# get the absolute path of this script
subt_params_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# remove from config
remove_from_config() {
  # check if file exists
  if [ ! -f $1 ]; then
    return;
  fi
  # remove from path
  sed -i '/Exporting SubT roslauanch params/d' $1
  sed -i '/subt\.env/d' $1
}

# add to config
add_to_config() {
  # check if file exists
  if [ ! -f $1 ]; then
    return;
  fi
  # add the filepath to be sourced
  echo "# Exporting SubT roslauanch params" >> $1
  # arg1 is devel path, arg2 is the file that gets sourced
  echo ". $subt_params_dir/../params/subt.env" >> $1
}

# install
install() {
  echo "install $1"

  # remove any previous alias
  remove_from_config "$1"

  # script to source in zsh, bash ros setup file
  add_to_config "$1"

  echo "SubT Launch Environment Variables Installed."
}

# uninstall
uninstall() {
  echo "uninstall $1"

  # remove any previous alias
  remove_from_config "$1"

  echo "SubT Launch Environment Variables Uninstalled."
}


### perform the install/uninstall ###
if [ "$1" == "--install" ]; then
  install $2
elif [ "$1" == "--uninstall" ]; then
  uninstall $2
elif [ "$1" == "--help" ]; then
  echo -e "$YELLOW$usage_msg$DEFCOL\n";
else
  echo -e "$YELLOW$usage_msg$DEFCOL\n";
fi
validate "subt-params-install script install or uninstall failed."
