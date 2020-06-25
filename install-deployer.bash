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
DEPLOYER_SCRIPTS_PATH="/operations/deploy/scripts/deployer_scripts/"

# //////////////////////////////////////////////////////////////////////////////
SRC_DIR=$(pwd)
# get the home directory
homedir=~
eval homedir=$homedir

# remove from config
remove_from_config() {
  # check if file exists
  if [ ! -f /$homedir/.$1 ]; then
    return;
  fi
  # remove title
  sed -i '/Deploy Setup/d' /$homedir/.$1
  # remove alias
  sed -i '/deploy-cd/d' /$homedir/.$1
  sed -i '/deploy-cd-azure/d' /$homedir/.$1
  sed -i '/deploy-azure-limits-eastus2/d' /$homedir/.$1
  sed -i '/deploy-azure-limits-eastus/d' /$homedir/.$1
  sed -i '/deploy-vpn-ca-cert/d' /$homedir/.$1
  # remove paths
  sed -i '/SUBT_PATH/d' /$homedir/.$1
  sed -i '/SUBT_DOCKER_PATH/d' /$homedir/.$1
  sed -i '/\$SUBT_DOCKER_PATH/d' /$homedir/.$1
  sed -i '/DEPLOY_UTILS_PATH/d' /$homedir/.$1
  sed -i '/\$DEPLOY_UTILS_PATH/d' /$homedir/.$1
}

# add to config
add_to_config() {
  # check if file exists
  if [ ! -f /$homedir/.$1 ]; then
    return;
  fi
  # add title
  echo "# == Deploy Setup ==" >> /$homedir/.$1
  # add alias
  echo "alias deploy-cd=\"cd $SRC_DIR/\"" >> /$homedir/.$1
  echo "alias deploy-cd-azure=\"cd $SRC_DIR/operations/deploy/azurebooks/subt/\"" >> /$homedir/.$1
  echo "alias deploy-azure-limits-eastus='az vm list-usage --location \"East US\" -o table | grep \"Total Regional vCPUs\"'" >> /$homedir/.$1
  echo "alias deploy-azure-limits-eastus2='az vm list-usage --location \"East US 2\" -o table | grep \"Total Regional vCPUs\"'" >> /$homedir/.$1
  echo "alias deploy-vpn-ca-cert='openssl x509 -in caCert.pem -outform der | base64 -w0 ; echo'" >> /$homedir/.$1
  # add paths
  echo "export SUBT_PATH=$SRC_DIR/" >> /$homedir/.$1
  echo "export SUBT_DOCKER_PATH=$SRC_DIR/operations/deploy/docker/scripts/" >> /$homedir/.$1
  echo "export DEPLOY_UTILS_PATH=$SRC_DIR/operations/utils/sysadmin/" >> /$homedir/.$1
  echo "export PATH=\$PATH:\$SUBT_PATH:\$SUBT_DOCKER_PATH:\$DEPLOY_UTILS_PATH" >> /$homedir/.$1
}

# install deployer's python scripts
install_deployer_py_scripts() {
  # install python scripts
  cd $SRC_DIR/$DEPLOYER_SCRIPTS_PATH
  pwd
  python setup.py install --user
  validate "builder install failed."
  git clean -f -d
}

# uninstall deployer's python scripts
uninstall_deployer_py_scripts() {
  cd $SRC_DIR/$DEPLOYER_SCRIPTS_PATH
  pwd
  python setup.py install --record egg-files.txt --user
  validate "builder uninstall failed."
  cat egg-files.txt | xargs rm -rf
  git clean -f -d
}

# install
install() {
  # update the submodules & install deployer python scripts
  git submodule update --init --recursive $SRC_DIR/operations
  install_deployer_py_scripts

  # remove any previous alias
  remove_from_config "zshrc"
  remove_from_config "bashrc"

  # script add to zsh, bash configs
  add_to_config "zshrc"
  add_to_config "bashrc"

  echo "subt scripts installed."
}

# uninstall
uninstall() {
  # remove deployer python scripts
  uninstall_deployer_py_scripts

  # remove any previous alias
  remove_from_config "zshrc"
  remove_from_config "bashrc"

  echo "subt scripts uninstalled."
}

### perform the install/uninstall ###
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
