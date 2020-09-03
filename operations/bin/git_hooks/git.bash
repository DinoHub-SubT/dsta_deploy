#!/usr/bin/env bash

# load header helper functions
. "$SUBT_PATH/operations/bin/.header.bash"
. "$SUBT_PATH/operations/bin/git_hooks/.header.bash"

# globals
GL_GIT_HOOKS_DIR=$SUBT_PATH/operations/bin/git_hooks/

# example:
# list all submodules
# git subt list -- lists all modules

# show the git subt status, no recursive?
# git subt status -ug -ua -c -l -s -d -b
# git subt status -ugv -uav -cmn -launch -sim -docker -basestation
# git subt status -r -ug # recursive
# git subt status -m base_node

# git subt fetch -r -ug -ua -c -l -s -d -b
# git subt fetch -r -ug -ua -c -l -s -d -b

# syncs all local branches with remote. i.e. does a hard reset
# git subt sync -r -ugv

# ??
# git subt pull -ugv
# git subt push -ugv

# git subt add /path/to/submodule

# git subt checkout -b branch name

# //////////////////////////////////////////////////////////////////////////////
# git subt main entrypoint, switch the subcommand
# //////////////////////////////////////////////////////////////////////////////
git_subt_main() {
  GL_TEXT_COLOR=$FG_LCYAN
  text_color "== Deployer Git Hooks =="

  # git status
  if chk_flag list $@; then
    shift # shift the 'list' subcommand
    ($GL_GIT_HOOKS_DIR/list.bash $@)

  # git fetch
  elif chk_flag fetch $@; then
    . $GL_GIT_HOOKS_DIR/fetch.bash
  fi

  GL_TEXT_COLOR=$FG_DEFAULT
}

# //////////////////////////////////////////////////////////////////////////////
# git wrapper, to enable 'subt' subcommand options.
# //////////////////////////////////////////////////////////////////////////////
git() {

  # execute 'git subt' subcommand
  if check_nth_flag 1 "subt" "$@"; then
    shift # shift the subt flag

    if chk_flag --help $@ || chk_flag help $@ || chk_flag -h $@; then
      GL_TEXT_COLOR=$FG_LCYAN
      text
      title "Usage: $(basename $0) subt [command] [<arg>] "
      text_color "Commands:"
      text_color "      status: show the working tree status for all level git repos."
      text_color "      fetch:  download objects and refs for all level git repos."
      text_color "      checkout: TODO"
      text_color "      sync: TODO"
      text_color "      pull: TODO"
      text_color "      push: TODO"
      text_color "      -h, --help, help  : shows usage help for 'git subt'"
      text_color "For more help, please see the README.md or wiki."
      GL_TEXT_COLOR=$FG_DEFAULT
      return 0
    fi

    # execute git subt main entrypoint
    git_subt_main $@

    # shift the subt arguments (they are standard for all 'git subt' subcommands)
    for arg do
      shift
      chk_eq $arg "-a" || chk_eq $arg "-bs" || chk_eq $arg "-cm" || \
        chk_eq $arg "-sim" || chk_eq $arg "-uav" || chk_eq $arg "-ugv" \
        && continue
      set -- "$@" "$arg"
    done
  fi

  # execute actual git command
  if ! chk_flag list $@; then
    command git "$@"
  fi
}
