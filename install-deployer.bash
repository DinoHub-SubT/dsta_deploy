#!/usr/bin/env bash

# check if operations (submodule) is empty. if so, clone it.
if [ -z "$(ls -A $(pwd)operations/scripts/header.sh)" ]; then
  git submodule deinit -f operations
  git submodule update --init --recursive operations
fi

# //////////////////////////////////////////////////////////////////////////////
# @brief globals, includes & help
# //////////////////////////////////////////////////////////////////////////////
GL_SRC_DIR="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
GL_OP_DIR="$GL_SRC_DIR/operations/"
GL_SUBT_ENV_DIR=$HOME/.dsta/
GL_RC=$GL_SUBT_ENV_DIR/subtrc.bash
GL_CMPL_SRC_DIR=$GL_SUBT_ENV_DIR/.completion/
GL_DEPLOYER_PATH="/operations/deployer/"
GL_RC_SUBT_CFG=$GL_SUBT_ENV_DIR/subt_config.bash
GL_RC_USER_CFG=$GL_SUBT_ENV_DIR/user_config.bash
GL_RC_ANI_CFG=$GL_SUBT_ENV_DIR/ansible_config.yaml
GL_RC_TERRA_CFG=$GL_SUBT_ENV_DIR/terraform_config.yaml

# load header helper functions
. "$GL_SRC_DIR/operations/scripts/header.sh"
. "$GL_SRC_DIR/operations/scripts/formatters.sh"

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

# //////////////////////////////////////////////////////////////////////////////
# @brief create the anisble rc configuration file
# //////////////////////////////////////////////////////////////////////////////
create_ani_cfg() {
  # remove the previously created file
  rm_file $GL_RC_ANI_CFG

  # copy over the template file to the subt configuration path
  cp $GL_OP_DIR/ansiblebooks/vars/user.template $GL_RC_ANI_CFG
}

# //////////////////////////////////////////////////////////////////////////////
# @brief create the subt rc configuration file
# //////////////////////////////////////////////////////////////////////////////
create_subt_cfg() {
  # remove the previously created file
  rm_file $GL_RC_SUBT_CFG

  # add deployer title
  write $GL_RC_SUBT_CFG "#!/usr/bin/env bash"
  write $GL_RC_SUBT_CFG
  write $GL_RC_SUBT_CFG "# == SubT Configuration == "
  write $GL_RC_SUBT_CFG

  # general paths
  write $GL_RC_SUBT_CFG "# -- General Paths -- "
  write $GL_RC_SUBT_CFG
  write $GL_RC_SUBT_CFG "export SUBT_PATH=$GL_SRC_DIR/"
  write $GL_RC_SUBT_CFG "export SUBT_DOCKERFILE_PATH=$GL_OP_DIR/docker/dockerfiles/"
  write $GL_RC_SUBT_CFG "export SUBT_OPERATIONS_PATH=$GL_OP_DIR/"
  write $GL_RC_SUBT_CFG "export SUBT_SCENARIOS_PATH=$GL_OP_DIR/scenarios/"
  write $GL_RC_SUBT_CFG "export SUBT_CONFIGURATION_PATH=$GL_SUBT_ENV_DIR/"
  write $GL_RC_SUBT_CFG

  # deployer paths
  write $GL_RC_SUBT_CFG "# -- Deployer Paths -- "
  write $GL_RC_SUBT_CFG
  write $GL_RC_SUBT_CFG "export DEPLOYER_PATH=$GL_OP_DIR/deployer/"
  write $GL_RC_SUBT_CFG "export DEPLOYER_EXPORT_FILEPATH=$GL_CMPL_SRC_DIR/"
  write $GL_RC_SUBT_CFG "export DEPLOYER_BASHRC_FILEPATH=.dsta/subtrc.bash"
  write $GL_RC_SUBT_CFG

  # script paths
  write $GL_RC_SUBT_CFG "# -- Script Paths -- "
  write $GL_RC_SUBT_CFG
  write $GL_RC_SUBT_CFG "export SUBT_GENERAL_SCRIPTS=$GL_OP_DIR/scripts/"
  write $GL_RC_SUBT_CFG "export SUBT_DOCKER_SCRIPTS=$GL_OP_DIR/docker/scripts/"
  write $GL_RC_SUBT_CFG "export SUBT_TERRAFORM_SCRIPTS=$GL_OP_DIR/azurebooks/scripts/"
  write $GL_RC_SUBT_CFG "export SUBT_PLAYBOOKS_SCRIPTS=$GL_OP_DIR/ansiblebooks/scripts/"
  write $GL_RC_SUBT_CFG "export SUBT_DEPLOY_SCRIPTS=$GL_OP_DIR/scripts/"
  write $GL_RC_SUBT_CFG "export DEPLOYER_SCRIPTS=$GL_OP_DIR/deployer/bin/"
  write $GL_RC_SUBT_CFG

  # add all relevant paths to env PATH
  write $GL_RC_SUBT_CFG "# -- Append to PATH -- "
  write $GL_RC_SUBT_CFG "export PATH=\$PATH:\$SUBT_PATH"
  write $GL_RC_SUBT_CFG "export PATH=\$PATH:\$SUBT_DOCKER_SCRIPTS"
  write $GL_RC_SUBT_CFG "export PATH=\$PATH:\$SUBT_TERRAFORM_SCRIPTS"
  write $GL_RC_SUBT_CFG "export PATH=\$PATH:\$SUBT_PLAYBOOKS_SCRIPTS"
  write $GL_RC_SUBT_CFG "export PATH=\$PATH:\$SUBT_GENERAL_SCRIPTS"
  write $GL_RC_SUBT_CFG "export PATH=\$PATH:\$SUBT_DEPLOY_SCRIPTS"
  write $GL_RC_SUBT_CFG "export PATH=\$PATH:\$DEPLOYER_SCRIPTS"
  write $GL_RC_SUBT_CFG
}

# //////////////////////////////////////////////////////////////////////////////
# @brief create the subt, user configuration file
# //////////////////////////////////////////////////////////////////////////////
create_user_cfg() {
  # remove the previously created file
  rm_file $GL_RC_USER_CFG

  # add deployer title
  write $GL_RC_USER_CFG "#!/usr/bin/env bash"
  write $GL_RC_USER_CFG
  write $GL_RC_USER_CFG "# == SubT User Configurations == "
  write $GL_RC_USER_CFG "# - if not sure about the configuration setup, leave the default values."
  write $GL_RC_USER_CFG

  # add all relevant paths to env PATH
  write $GL_RC_USER_CFG "# -- General Configurations -- "
  write $GL_RC_USER_CFG

  write $GL_RC_USER_CFG "# Set to the cloned workspace name"
  write $GL_RC_USER_CFG "export DEPLOYER_WS_NAME=$(basename $(dirname $GL_SRC_DIR))"
  write $GL_RC_USER_CFG

  write $GL_RC_USER_CFG "# Set to 'true' only if your system has an nvidia graphics driver. Otherwise set to 'false'."
  write $GL_RC_USER_CFG "export USE_NVIDIA_DRIVER=true"
  write $GL_RC_USER_CFG

  write $GL_RC_USER_CFG "# Set to 'true' only if you want to use the enhanced docker gpu images. Otherwise set to 'false'."
  write $GL_RC_USER_CFG "# - if the above nvidia_driver is false, this will always be disabled."
  write $GL_RC_USER_CFG "export USE_ENHANCED_GPU_DOCKER_IMAGES=false"
  write $GL_RC_USER_CFG

  write $GL_RC_USER_CFG "# Set the deployer's books path "
  write $GL_RC_USER_CFG "export DEPLOYER_BOOKS_PATH=$GL_OP_DIR/deploybooks/"
  write $GL_RC_USER_CFG "export DEPLOYER_BOOKS_EXTEND_PATH=$GL_OP_DIR/deploybooks/"
  write $GL_RC_USER_CFG

  write $GL_RC_USER_CFG "# Set to the number of parallelized cores to run deployer matches"
  write $GL_RC_USER_CFG "export DEPLOYER_PARALLELIZE_MATCHES_JOBS=2"
  write $GL_RC_USER_CFG

  # add general alias commands
  write $GL_RC_USER_CFG "# -- Deployer Configurations -- "
  write $GL_RC_USER_CFG
  write $GL_RC_USER_CFG "export ENABLE_UGV_HARDWARE=false"
  write $GL_RC_USER_CFG "export ENABLE_SLAM=false"
  write $GL_RC_USER_CFG

  # add general alias commands
  write $GL_RC_USER_CFG "# -- General Alias -- "
  write $GL_RC_USER_CFG
  write $GL_RC_USER_CFG "alias cdd=\"cd $GL_SRC_DIR/\""
  write $GL_RC_USER_CFG
}

create_terra_cfg() {
  # remove the previously created file
  rm_file $GL_RC_TERRA_CFG

  # install the terraform configurations
  source $GL_RC_SUBT_CFG
  ($GL_SRC_DIR/operations/azurebooks/scripts/install-terraform-current.sh -c $GL_SUBT_ENV_DIR)
  if last_command_failed; then
    error "There was an error with the installer script. Please notify maintainer."
    exit_failure
  fi

  # add deployer title
  write $GL_RC_TERRA_CFG "#!/usr/bin/env bash"
  write $GL_RC_TERRA_CFG
  write $GL_RC_TERRA_CFG "# == SubT Terraform Configurations == "
  write $GL_RC_TERRA_CFG "# - if not sure about the configuration setup, leave the default values."
  write $GL_RC_TERRA_CFG
  write $GL_RC_TERRA_CFG "source $GL_SUBT_ENV_DIR/terraform_id.bashrc"
  write $GL_RC_TERRA_CFG "source $GL_SUBT_ENV_DIR/terraform_flags.bashrc"
  write $GL_RC_TERRA_CFG

  write $GL_RC_TERRA_CFG "# -- Terraform Alias -- "
  write $GL_RC_TERRA_CFG
  write $GL_RC_TERRA_CFG "alias subtu-azure-limits-eastus='az vm list-usage --location \"East US\" -o table | grep \"Total Regional vCPUs\"'"
  write $GL_RC_TERRA_CFG "alias subtu-azure-limits-eastus2='az vm list-usage --location \"East US 2\" -o table | grep \"Total Regional vCPUs\"'"
  write $GL_RC_TERRA_CFG "alias subtu-vpn-ca-cert='openssl x509 -in caCert.pem -outform der | base64 -w0 ; echo'"
  write $GL_RC_TERRA_CFG
}

# //////////////////////////////////////////////////////////////////////////////
# @brief create the subt rc file
# //////////////////////////////////////////////////////////////////////////////
create_subtrc() {
  # remove the previously created file
  rm_file $GL_RC

  write $GL_RC "#!/usr/bin/env bash"
  write $GL_RC

  # add deployer title
  write $GL_RC "# == SubT Deployer =="
  write $GL_RC

  # source configuration
  write $GL_RC "# SubT Configuration "
  write $GL_RC "source $GL_RC_SUBT_CFG"
  write $GL_RC "source $GL_RC_USER_CFG"
  write $GL_RC "source $GL_RC_TERRA_CFG"
  write $GL_RC

  # add auto-completion
  write $GL_RC "# SubT Deployer Autocompletion "
  write $GL_RC "source $GL_OP_DIR/scripts/automate/subt" >> $GL_RC
  write $GL_RC
}

# //////////////////////////////////////////////////////////////////////////////
# @brief add git hooks
# //////////////////////////////////////////////////////////////////////////////
add_hooks() {
  # make the subt hook's destination direcotry
  local dest_dirpath=$GL_SRC_DIR/.git/hooks/
  mkdir -p $dest_dirpath/subt

  # get the source hooks path, to copy over
  local src_dirpath=$GL_OP_DIR/scripts/hooks/active

  # go through each hooks file, add the subt hooks
  for filepath in $src_dirpath/*; do

    # copy the src hook to the dest hook path
    cp $filepath $dest_dirpath/subt

    # get the prefix, hook name
    local basename=$(basename -- "$filepath")
    local hook="${basename%.*}"

    # create & setup the destination hooks filepath
    local dest_filepath=$dest_dirpath/${hook}

    # create & initialize top hoooks file if it does not exist
    if ! file_exists $dest_filepath; then
      touch $dest_filepath
      chmod +x $dest_filepath
      write $dest_filepath "#!/usr/bin/env bash"
      write $dest_filepath
    fi

    # remove any previous sources
    sed -i "/${basename}/d" $dest_filepath

    # add the hook to git's top level hook path
    write $dest_filepath "source $filepath"
  done

  # exit, if rc does not exist
  if ! file_exists $GL_RC; then
    warning "cannot add git hooks, $GL_RC does not exist"
    return;
  fi

  # remove any previous git hooks
  # rm_hooks
}

# //////////////////////////////////////////////////////////////////////////////
# @brief remove git hooks
# //////////////////////////////////////////////////////////////////////////////
rm_hooks() {
  # make the subt hook's destination direcotry
  local dest_dirpath=$GL_SRC_DIR/.git/hooks/
  rm_dir $dest_dirpath/subt

  # get the source hooks path, to copy over
  local src_dirpath=$GL_OP_DIR/scripts/hooks/

  # go through each hooks files, remove any references to subt hooks
  for filepath in $src_dirpath/*; do
    echo "filepath is: $filepath"

    # get the prefix, hook name
    local basename=$(basename -- "$filepath")

    # remove any previous sources
    sed -i "/${basename}/d" $dest_filepath
  done
}

# //////////////////////////////////////////////////////////////////////////////
# @brief export the ansible version
# //////////////////////////////////////////////////////////////////////////////
export_versions() {
  # create the version file
  local versionf=$GL_SUBT_ENV_DIR//version
  touch $versionf

  # remove previous version file
  rm $versionf

  # check if file exists, do not copy over?

  # create all the version files
  write $versionf "$(cat $GL_OP_DIR/ansiblebooks/.version)"
  write $versionf "$(cat $GL_OP_DIR/ansiblebooks/.version)"
}

# //////////////////////////////////////////////////////////////////////////////
# @brief add zsh/bashrc installs
# //////////////////////////////////////////////////////////////////////////////
add_to_rc() {
  local rc=/$HOME/.$1
  # exit, if rc does not exist
  if ! file_exists $rc; then
    return;
  fi
  # source in rc
  echo "source $GL_RC" >> $rc
}

# //////////////////////////////////////////////////////////////////////////////
# @brief remove zsh/bashrc installs
# //////////////////////////////////////////////////////////////////////////////
rm_from_rc() {
  local rc=/$HOME/.$1
  # exit, if rc does not exist
  if ! file_exists $rc; then
    return;
  fi
  # remove source from rc
  sed -i '/subtrc/d' $rc
}

# //////////////////////////////////////////////////////////////////////////////
# @brief install deployer's autocompletion matches
# //////////////////////////////////////////////////////////////////////////////
install_deployer_cmpl_matches() {
  text "...installing tab-completion"

  # go back to top level deploy directory
  cd $GL_SRC_DIR

  # export the configuration files
  source $GL_RC_SUBT_CFG
  source $GL_RC_USER_CFG

  # create the completion directory
  mkdir -p $GL_CMPL_SRC_DIR

  # install the deployer git commands
  deployer -s git --export git \
    -f  git.all. .other \
        .basestation.meta.sc .basestation.sub.sc \
        .common.meta.sc .common.sub.sc \
        .subt_launch.meta.sc .subt_launch.sub.sc \
        .simulation.meta.sc .simulation.sub.sc \
        .perception.meta.sc .perception.sub.sc \
        .ugv.meta.sc .ugv.sub.sc \
        .uav.meta.sc .uav.sub.sc \
        all. \
        local.checkout local.clone local.pull local.reset \
        local.rm local.clean local.ignore local.unignore

  # install the deployer tutorial commands
  deployer -s tutorial --export tutorial -f tutorial.

  # install the localhost deployer commands
  deployer -s local --export deployer \
    -f  .local.plt.ugv .azure.plt.ugv .robot.plt.ugv .perception.wifi .plt \
        .basestation.ws .perception.ws .uav.core.ws .uav.wifi.ws .ugv.core.ws .ugv.nuc.ws \
        .uav.simulation .ugv.simulation .robot.uav .uav.ds.scenarios .scenarios.uav \
        .scenarios.ugv .uav.ds.scenarios.uav .ws.catkin.catkin .host \
        .compose .container .scenarios .actions .workspace .robots .sync .run .docker_registry .ws

  # append the azure deployer commands
  deployer -s azure -ap --export deployer \
    -f  .local.plt.ugv .azure.plt.ugv .robot.plt.ugv .perception.wifi .plt \
       .basestation.ws .perception.ws .uav.core.ws .uav.wifi.ws .ugv.core.ws .ugv.nuc.ws \
       .ugv.nuc.core.ws .scenarios.ugv .uav.ds.scenarios.uav .scenarios.uav \
       .ugv.split .ugv.ppc.ws .ugv.nuc.ws .ugv.xavier.ws .ugv.ppc .ugv.nuc .ugv.xavier \
       .uav.simulation .ugv.simulation .robot.uav .uav.ds.scenarios .host \
       .compose .container .scenarios .actions .workspace .robots .sync .run .docker_registry .ws .rsync

  # append the localhost deployer commands
  deployer -s robots -ap --export deployer \
   -f  .local.plt.ugv .azure.plt.ugv .robot.plt.ugv .perception.wifi .plt \
       .basestation.ws .perception.ws .uav.core.ws .uav.wifi.ws .ugv.core.ws \
       .ugv.nuc.core.ws .scenarios.ugv .uav.ds.scenarios.uav .scenarios.uav \
       .ugv.split .ugv.ppc.ws .ugv.nuc.ws .ugv.xavier.ws .ugv.ppc .ugv.nuc .ugv.xavier \
       .uav.simulation .ugv.simulation .robot.uav .uav.ds.scenarios .host \
       .scenarios .actions .compose .robots .sync .container .run .docker_registry .ws .rsync

  # add any propogated section keys
  deployer -ex deployer -ap -pr catkin.build catkin.clean
  deployer -ex deployer -pr docker skel_to transfer

  # hack-ugly, add the 'all', to aggregate all git commands
  echo "all.reset" >> $DEPLOYER_EXPORT_FILEPATH/git.cmpl
  echo "all.pull" >> $DEPLOYER_EXPORT_FILEPATH/git.cmpl
  echo "all.ignore" >> $DEPLOYER_EXPORT_FILEPATH/git.cmpl
  echo "all.unignore" >> $DEPLOYER_EXPORT_FILEPATH/git.cmpl
  echo "all.rm" >> $DEPLOYER_EXPORT_FILEPATH/git.cmpl
  echo "all.clean" >> $DEPLOYER_EXPORT_FILEPATH/git.cmpl

  # verify the auto-completion files are created
  if ! file_exists $GL_CMPL_SRC_DIR/deployer.cmpl; then
    error "deployer auto-completion files failed, please notify maintainer."
    exit_failure
  fi
  # verify the auto-completion files are created
  if ! file_exists $GL_CMPL_SRC_DIR/git.cmpl; then
    error "deployer auto-completion files failed, please notify maintainer."
    exit_failure
  fi
}

# //////////////////////////////////////////////////////////////////////////////
# @brief install deployer's python scripts
# //////////////////////////////////////////////////////////////////////////////
install_deployer_py_scripts() {
  # clone all operation submodules
  git submodule update --init --recursive $GL_OP_DIR

  # install python scripts
  cd $GL_SRC_DIR/$GL_DEPLOYER_PATH
  python setup.py install --user
  # validate python installed deployer
  if last_command_failed; then
    "deploy builder install failed."
    exit_failure
  fi
  git clean -f -d
}

# //////////////////////////////////////////////////////////////////////////////
# @brief uninstall deployer's python scripts
# //////////////////////////////////////////////////////////////////////////////
uninstall_deployer_py_scripts() {
  # re-initialize the submodules in case there was a corrupted clone previously
  git submodule update --init --recursive $GL_OP_DIR

  # remove any installed deployerbooks
  cd $GL_SRC_DIR/$GL_DEPLOYER_PATH
  python setup.py install --record egg-files.txt --user
  cat egg-files.txt
  cat egg-files.txt | xargs rm -rf
  git clean -f -d

  # remove the deployer submodule
  cd $GL_SRC_DIR
  git submodule deinit -f $GL_OP_DIR
  if last_command_failed; then
    "deploy builder uninstall failed."
    exit_failure
  fi
}

##
# install thirdparty libraries
##
install_thirdparty() {
  # deployer dependencies
  sudo apt-get update
  sudo apt install -y --no-install-recommends git python2.7 python-setuptools python-pip
  pip2 install wheel --user
  pip2 install setuptools PyYAML pexpect --user

  sudo apt install -y python3-dev python3-pip libffi-dev libssl-dev git-lfs
  sudo -H pip3 install wheel setuptools

  # install ansible
  sudo apt update
  sudo apt install -y software-properties-common
  sudo apt-add-repository --yes --update ppa:ansible/ansible
  sudo apt install -y ansible
}

# //////////////////////////////////////////////////////////////////////////////
# @brief install all deploy operations tools and configuration files
# //////////////////////////////////////////////////////////////////////////////
install() {
  # update the submodules & install deployer python scripts
  install_deployer_py_scripts

  # create the subt environment config file
  mkdir -p $GL_SUBT_ENV_DIR

  # create the subt rc files
  text
  text ...creating configuration files

  create_subtrc
  create_subt_cfg
  create_user_cfg
  create_ani_cfg
  create_terra_cfg

  # TODO: complete the git hooks (i.e. validate versions)
  # add_hooks

  # remove any previous deployer env setup
  rm_from_rc "bashrc"
  if file_exists "$(realpath ~/.zshrc)"; then
    rm_from_rc "zshrc"
  fi

  # add deployer env setup
  add_to_rc "bashrc"
  if file_exists "$(realpath ~/.zshrc)"; then
    add_to_rc "zshrc"
  fi

  install_deployer_cmpl_matches

  text
  warning "To activate deployer, source your rc: 'source ~/.bashrc' or 'source ~/.zshrc'"
  text
  text "SubT scripts installed."
}

# //////////////////////////////////////////////////////////////////////////////
# @brief uninstall all deploy operations tools and configuration files
# //////////////////////////////////////////////////////////////////////////////
uninstall() {
  # remove deployer python scripts
  uninstall_deployer_py_scripts

  # -- attempt to have "safe uninstall"...obviously not the best way to do things.
  text
  text ...removing configuration files

  # remove the deploy autocompletion files
  if dir_exists $GL_CMPL_SRC_DIR; then
    rm -rf $GL_CMPL_SRC_DIR/*
    rm_dir $GL_CMPL_SRC_DIR
  else
    warning "Cannot remove $GL_CMPL_SRC_DIR directory does not exist"
  fi

  # remove the rc and the configuration files
  if dir_exists $GL_SUBT_ENV_DIR; then
    rm -rf $GL_SUBT_ENV_DIR/*
    rm_dir $GL_SUBT_ENV_DIR
  else
    warning "Cannot remove $GL_SUBT_ENV_DIR directory does not exist"
  fi

  # remove any previous deployer env setup
  rm_from_rc "bashrc"
  if file_exists "$(realpath ~/.zshrc)"; then
    rm_from_rc "zshrc"
  fi

  text
  text "SubT scripts uninstalled."
}

# //////////////////////////////////////////////////////////////////////////////
# @brief script main entrypoint
# //////////////////////////////////////////////////////////////////////////////
title "Deployer Install\n"

# push script path
__dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
pushd $__dir

if chk_flag -t $@ || chk_flag --thirdparty $@; then
  install_thirdparty
fi

### perform the install/uninstall ###
if chk_flag -i $@ || chk_flag --install $@; then
  install
elif chk_flag -u $@ || chk_flag --uninstall $@; then
  uninstall
else
  error "Invalid argument. Please use --help to see available arguments."
fi

# cleanup & exit
exit_pop_success
