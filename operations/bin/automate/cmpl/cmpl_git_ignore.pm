#!/usr/local/bin/perl

package cmpl_git_ignore;
use Exporter;

# //////////////////////////////////////////////////////////////////////////////
# @brief export modules
# //////////////////////////////////////////////////////////////////////////////

our @ISA= qw( Exporter );

# these CAN be exported.
our @EXPORT_OK = qw(
  @_git_ignore
  $_deployer_ignore_help
  @_git_ignore_help
);

# these are exported by default.
our @EXPORT = qw(
  @_git_ignore
  $_deployer_ignore_help
  @_git_ignore_help
);

our (
  @_git_ignore,
  $_deployer_ignore_help,
  @_git_ignore_help
);

# //////////////////////////////////////////////////////////////////////////////
# @brief deployer (git ignore) alias as arrays & associated helps
# //////////////////////////////////////////////////////////////////////////////
@_git_ignore     = (
  "basestation",
  "common",
  "perception",
  "subt_launch",
  "simulation",
  "ugv",
  "uav",
);

$_deployer_ignore_help = ("
About: 00... == ignore pre-defined files from git index ==
About: 01... ignores files from git index, so you will not see them as uncommitted changes.
About: 02... - these files are predefined by the maintainer -- usually auto-generated files
About: 03... - use 'ignore', it will reduce the amount of unnecessary files to track from the 'git status'
About: 04...
About: 05... == Optional Flags ==
About: 06...
About: 07...   -p           : preview the deployer commands that will be run
About: 08...   -verbose     : show the exact (verbose) bash commands that will run
About: 09...
About: 10... == Your Options Are ==
About: 11...
basestation        : ~/deploy_ws/src/basestation
common             : ~/deploy_ws/src/common
perception         : ~/deploy_ws/src/perception
simulation         : ~/deploy_ws/src/simulation
subt_launch        : ~/deploy_ws/src/subt_launch
ugv                : ~/deploy_ws/src/ugv
uav                : ~/deploy_ws/src/uav
");

# @brief assign help keys to usage messages as hashmap -- hack: convert array to hashmap
@_git_ignore_help = ({
  id      => "ignore",
  help    => $_deployer_ignore_help,
});
