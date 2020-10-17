#!/usr/local/bin/perl

# //////////////////////////////////////////////////////////////////////////////
# @brief import modules
# //////////////////////////////////////////////////////////////////////////////

use Cwd qw(abs_path);
use FindBin;
use lib abs_path("$FindBin::Bin/../lib");

use File::Basename;
use lib dirname (__FILE__);

use cmpl_header;
use cmpl_deployer;
use cmpl_git_clone;
use cmpl_git_reset;
use cmpl_git_rm;
use cmpl_git_clean;
use cmpl_git_pull;

# //////////////////////////////////////////////////////////////////////////////
# @brief [TAB] autocompletion matcher functionality
# //////////////////////////////////////////////////////////////////////////////

# @brief covert the array to hashmap
my %_deployer_help_hash = map {
  $_->{id} => { help => $_->{help} }
} @_deployer_help_array;

my %_git_clone_help_hash = map {
  $_->{id} => { help => $_->{help} }
} @_git_clone_help;

my %_git_reset_help_hash = map {
  $_->{id} => { help => $_->{help} }
} @_git_reset_help;

my %_git_rm_help_hash = map {
  $_->{id} => { help => $_->{help} }
} @_git_rm_help;

my %_git_clean_help_hash = map {
  $_->{id} => { help => $_->{help} }
} @_git_clean_help;

my %_git_pull_help_hash = map {
  $_->{id} => { help => $_->{help} }
} @_git_pull_help;


# //////////////////////////////////////////////////////////////////////////////
# @brief regex functionality
# //////////////////////////////////////////////////////////////////////////////
# @brief match the suffix of the target token (for deployer help tab-complete)
sub help_sregex {
  my ($_target, $_i) = @_;
  $_target =~ qr/(\.[^.]+){$_i}$/;
  return $&;
}
# @brief match the prefix of the target token (for deployer help tab-complete)
sub help_pregex {
  my ($_target, $_i) = @_;
  $_target =~ qr/^([^.].*\.)/;
  return $&;
}

# @brief match the suffix of the target token (for deployer tab-complete)
sub sregex {
  my ($_target,  $_suffix) = @_;
  my $_regex="(?<=^$_target).*";
  $_suffix =~ m/$_regex/;
  return $&;
}

# @brief match the prefix of the target token (for deployer tab-complete)
sub pregex {
  my ($_prefix) = @_;
  my $_regex='^([^\.]+)';
  $_prefix =~ qr/$_regex/;
  return $&;
}

# @brief deployer regex matcher, main entrypoint
sub deploy_matcher {
  my ($_target, @_arr) = @_, $_result;
  foreach my $_deploy (@_arr) {
    my $_smatch = sregex($_target, $_deploy);
    if (! $_smatch eq "") {
      my $_pmatch = pregex($_smatch);
      # result:
      # -- given target is partial match, append target to result
      # -- add trailing '.' unless last token -- last token is when suffix & prefix match
      # $_result = $_smatch eq $_pmatch ? "$_result $_target$_pmatch" : "$_result $_target$_pmatch.";
      $_match = $_smatch eq $_pmatch ? "$_target$_pmatch" : "$_target$_pmatch.";
      $_result = "$_result $_match"
    }
  }
  return $_result;
}

# @brief match the deployer help message id with its usage string message
sub find_deployer_help_usage {
  my ($_str, %_help_usage) = @_, $_result;

  foreach my $_help (keys %_help_usage) {
    if ( $_help eq $_str ) {
      my $_usage = $_help_usage{$_help}->{help};
      if (! $_usage eq "") { return $_usage; }
    }
  }
  return;
}

# @brief match the deployer help usage message
sub deployer_help_matcher {
  my ($_target, %_help_usage) = @_, $_match;
  my $_prefix = help_pregex($_target);  # get the largest prefix (i.e. all tokens before the last '.')
  remove_trail_dot($_prefix);           # remove trailing '.'
  # find the first suffix of given tab-completed token
  my $_dot_counter=1;
  my $_suffix = help_sregex($_prefix, $_dot_counter);
  while ( ! $_suffix eq "" ) {  # get the next suffix, increasing the token by the next suffix
    remove_lead_dot($_suffix);  # remove leading '.'
    # find the help associated with the suffix
    my $_usage = find_deployer_help_usage($_suffix, %_help_usage);
    # return help usage message -- if usage message was matched
    if (! $_usage eq "") { return $_usage; }
    # set the next suffix
    $_suffix = help_sregex($_prefix, ++$_dot_counter);
  }
  if ($_prefix eq "") { $_prefix = $_target; }
  return find_deployer_help_usage($_prefix, %_help_usage);
}

# @brief match the suffix of the target token
sub gregex {
  my ($_target,  $_str) = @_;
  my $_regex="^$_target.*.*?";
  $_str =~ m/$_regex/;
  return $&;
}

# @brief general matcher (i.e. non deployer commands)
sub general_matcher {
  my ($_target, @_subcommands) = @_, $_result;
  foreach my $_check (@_subcommands) {
    my $_match = gregex($_target, $_check);
    if (! $_match eq "") {
      $_result="$_result $_match";
    }
  }
  return $_result
}

# //////////////////////////////////////////////////////////////////////////////
# @brief main entrypoint
# //////////////////////////////////////////////////////////////////////////////
my ($_func, $_target) = @ARGV;

# match subcommands for each top command type
if (chk_flag($_func, "subt")  ) {
  print general_matcher($_target, @_subt);

} elsif (chk_flag($_func, "git")  ) {
  print general_matcher($_target, @_git);

} elsif (chk_flag($_func, "git_status")  ) {
  print general_matcher($_target, @_git_status);

} elsif (chk_flag($_func, "git_sync")  ) {
  print general_matcher($_target, @_git_sync);

} elsif (chk_flag($_func, "git_add")  ) {
  print general_matcher($_target, @_git_add);

} elsif (chk_flag($_func, "git_clone")  ) {
  my $_match = deploy_matcher($_target, @_git_clone);
  print deploy_matcher($_target);

} elsif (chk_flag($_func, "git_clone_help")  ) {
  print deployer_help_matcher($_target, %_git_clone_help_hash);

} elsif (chk_flag($_func, "git_reset")  ) {
  my $_match = deploy_matcher($_target, @_git_reset);
  print deploy_matcher($_target);

} elsif (chk_flag($_func, "git_reset_help")  ) {
  print deployer_help_matcher($_target, %_git_reset_help_hash);

} elsif (chk_flag($_func, "git_pull")  ) {
  my $_match = deploy_matcher($_target, @_git_pull);
  print deploy_matcher($_target);

} elsif (chk_flag($_func, "git_pull_help")  ) {
  print deployer_help_matcher($_target, %_git_pull_help_hash);

} elsif (chk_flag($_func, "git_rm")  ) {
  my $_match = deploy_matcher($_target, @_git_rm);
  print deploy_matcher($_target);

} elsif (chk_flag($_func, "git_rm_help")  ) {
  print deployer_help_matcher($_target, %_git_rm_help_hash);

} elsif (chk_flag($_func, "git_clean")  ) {
  my $_match = deploy_matcher($_target, @_git_clean);
  print deploy_matcher($_target);

} elsif (chk_flag($_func, "git_clean_help")  ) {
  print deployer_help_matcher($_target, %_git_clean_help_hash);

} elsif (chk_flag($_func, "cloud")  ) {
  print general_matcher($_target, @_cloud);

} elsif (chk_flag($_func, "cloud_terra")  ) {
  print general_matcher($_target, @_cloud_terra);

} elsif (chk_flag($_func, "cloud_ani")  ) {
  print general_matcher($_target, @_cloud_ani);

} elsif (chk_flag($_func, "tools")  ) {
  print general_matcher($_target, @_tools);

} elsif (chk_flag($_func, "deployer") ) {
  my $_match = deploy_matcher($_target, @_deployer);
  # print $_, "\n" for split ' ', "$_match";
  print deploy_matcher($_target);
} elsif (chk_flag($_func, "deployer_help") ) {
  print deployer_help_matcher($_target, %_deployer_help_hash);
} else {
  print "";  # return empy string on failure
}
