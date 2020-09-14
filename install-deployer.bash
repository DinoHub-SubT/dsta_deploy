#!/usr/bin/env bash

# globals
GL_DEPLOYER_PATH="/operations/deploy/deployer/"
GL_SRC_DIR=$(pwd)
GL_SUBT_ENV_DIR=$HOME/.subt/
GL_RC=$GL_SUBT_ENV_DIR/subtrc.bash

# load header helper functions
. "$GL_SRC_DIR/operations/deploy/azurebooks/scripts/header.sh"
. "$GL_SRC_DIR/operations/bin/.header.bash"

if chk_flag --help $@ || chk_flag help $@ || chk_flag -h $@; then
  GL_TEXT_COLOR=$FG_LCYAN
  text
  title "Usage: $(basename $0) [flag] "
  text_color  "Flags:"
  text_color "      -i, --install     : installs the deployer."
  text_color "      -u, --uninstall   : uninstalls the deployer."
  text_color "      -rg               : remove git hooks."
  text_color "      -h, --help, help  : shows usage message."
  text_color "Installs or uninstalls the builder script."
  text_color "For more help, please see the README.md or wiki."
  GL_TEXT_COLOR=$FG_DEFAULT
  exit_success
fi

# create the subt rc file
create_subtrc() {
  if file_exists $GL_RC; then
    rm $GL_RC
  fi
  echo "#!/usr/bin/env bash" >> $GL_RC
  echo >> $GL_RC
  # remove deployer title
  echo "# == SubT Deployer ==" >> $GL_RC
  # add deployer alias commands
  echo "alias cdd=\"cd $GL_SRC_DIR/\"" >> $GL_RC
  echo "alias cdd-azure=\"cd $GL_SRC_DIR/operations/deploy/azurebooks/subt/\"" >> $GL_RC
  echo "alias subtu-azure-limits-eastus='az vm list-usage --location \"East US\" -o table | grep \"Total Regional vCPUs\"'" >> $GL_RC
  echo "alias subtu-azure-limits-eastus2='az vm list-usage --location \"East US 2\" -o table | grep \"Total Regional vCPUs\"'" >> $GL_RC
  echo "alias subtu-vpn-ca-cert='openssl x509 -in caCert.pem -outform der | base64 -w0 ; echo'" >> $GL_RC
  # add deployer script paths
  echo "export SUBT_PATH=$GL_SRC_DIR/" >> $GL_RC
  echo "export SUBT_DOCKER_PATH=$GL_SRC_DIR/operations/deploy/docker/scripts/" >> $GL_RC
  echo "export DEPLOY_BIN_PATH=$GL_SRC_DIR/operations/bin/" >> $GL_RC
  echo "export TERRAFORM_UTILS_PATH=$GL_SRC_DIR/operations/deploy/azurebooks/scripts" >> $GL_RC
  echo "export PLAYBOOKS_UTILS_PATH=$GL_SRC_DIR/operations/deploy/robotbooks/scripts" >> $GL_RC
  echo "export DEPLOY_LAUNCH_PATH=$GL_SRC_DIR/subt_launch/" >> $GL_RC
  echo "export PATH=\$PATH:\$SUBT_PATH:\$SUBT_DOCKER_PATH:\$DEPLOY_BIN_PATH:\$TERRAFORM_UTILS_PATH:\$PLAYBOOKS_UTILS_PATH:\$DEPLOY_LAUNCH_PATH" >> $GL_RC
}

# remove git hooks
rm_hooks() {
  # exit, if zshrc or bashrc does not exist
  if ! file_exists $GL_RC; then
    warning "cannot remove git hooks, $GL_RC does not exist"
    return;
  fi

  # remove source from rc
  sed -i '/hooks\/subt/d' $GL_RC
}

# add git hooks
add_hooks() {
  # exit, if zshrc or bashrc does not exist
  if ! file_exists $GL_RC; then
    warning "cannot add git hooks, $GL_RC does not exist"
    return;
  fi

  # remove any previous git hooks
  rm_hooks

  # add git status hook
  echo >> $GL_RC
  echo "source $GL_SRC_DIR/operations/bin/automate/subt" >> $GL_RC
}

# remove zsh/bashrc installs
rm_from_rc() {
  local rc=/$HOME/.$1
  # exit, if zshrc or bashrc does not exist
  if ! file_exists $rc; then
    return;
  fi
  # remove source from rc
  sed -i '/subtrc/d' $rc
}

# add zsh/bashrc installs
add_to_rc() {
  local rc=/$HOME/.$1
  # exit, if zshrc or bashrc does not exist
  if ! file_exists $rc; then
    return;
  fi
  # source in rc
  echo "source $GL_RC" >> $rc
}

# install deployer's python scripts
install_deployer_py_scripts() {
  # install python scripts
  cd $GL_SRC_DIR/$GL_DEPLOYER_PATH
  python setup.py install --user
  validate "builder install failed."
  git clean -f -d
}

# uninstall deployer's python scripts
uninstall_deployer_py_scripts() {
  cd $GL_SRC_DIR/$GL_DEPLOYER_PATH
  python setup.py install --record egg-files.txt --user
  validate "builder uninstall failed."
  cat egg-files.txt | xargs rm -rf
  git clean -f -d
}

# install
install() {
  # update the submodules & install deployer python scripts
  # git submodule update --init --recursive $GL_SRC_DIR/operations
  # install_deployer_py_scripts

  # create the subt environment config file
  mkdir -p $GL_SUBT_ENV_DIR
  # remove subt rc file
  create_subtrc

  # remove any previous deployer env setup
  rm_from_rc "zshrc"
  rm_from_rc "bashrc"

  # add deployer env setup
  add_to_rc "zshrc"
  add_to_rc "bashrc"

  # add git hooks to subtrc
  add_hooks

  echo "SubT scripts installed."
}

# uninstall
uninstall() {
  # remove deployer python scripts
  # uninstall_deployer_py_scripts

  # remove subt rc file
  if file_exists $GL_RC; then
    rm $GL_RC
  fi

  # remove any previous deployer env setup
  rm_from_rc "zshrc"
  rm_from_rc "bashrc"

  echo "SubT scripts uninstalled."
}

# //////////////////////////////////////////////////////////////////////////////
# @brief: script main entrypoint
# //////////////////////////////////////////////////////////////////////////////
title "Deployer Install\n"

# push script path
__dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
pushd $__dir

### perform the install/uninstall ###
if chk_flag -i $@ || chk_flag --install $@; then
  install
  echo "install"
elif chk_flag -u $@ || chk_flag --uninstall $@; then
  uninstall
  echo "uninstall"
elif chk_flag -rg $@; then
  rm_hooks
else
  error "Invalid argument. Please use --help to see available arguments."
fi

# cleanup & exit
exit_on_success
