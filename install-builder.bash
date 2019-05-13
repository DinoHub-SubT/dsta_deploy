#!/usr/bin/env bash
# //////////////////////////////////////////////////////////////////////////////
# display usage
usage_msg="\
Usage: $(basename $0)

Options:
  --install
      install the builder script
  --uninstall
      install the builder script

  Installs or uninstalls the builder script.

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

# //////////////////////////////////////////////////////////////////////////////
SRC_DIR=$(pwd)

# install 
install() {
  git submodule init
  git submodule update $SRC_DIR/ci/ci_phase
  # validate "ci_phase submodule update failed."
  cd ci/ci_phase
  python setup.py install --user
  validate "builder install failed."
  git clean -f -d
  echo "alias deploy-docker-cd=\"cd $SRC_DIR/docker/scripts\"" >> ~/.bashrc
  echo "alias deploy-docker-cd=\"cd $SRC_DIR/docker/scripts\"" >> ~/.zshrc
  # add deploy
  echo "alias deploy-cd=\"cd $SRC_DIR/ \"" >> ~/.bashrc
  echo "alias deploy-cd=\"cd $SRC_DIR/ \"" >> ~/.zshrc
}

# uninstall
uninstall() {
  cd ci/ci_phase
  python setup.py install --record egg-files.txt --user
  validate "builder uninstall failed."
  cat egg-files.txt | xargs rm -rf
  git clean -f -d
  # TODO: remove alias, via pattern matching
}

# perform the install/uninstall
if [ "$1" == "--install" ]; then
  install
elif [ "$1" == "--uninstall" ]; then 
  uninstall
elif [ "$1" == "--help" ]; then 
  echo -e "$YELLOW$usage_msg$DEFCOL\n";
else
  echo -e "$YELLOW$usage_msg$DEFCOL\n";
fi
validate "builder install or uninstall failed."

# exit
exit_success

