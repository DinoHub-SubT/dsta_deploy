# Deploy Git Helpers

Deployer adds git helper tools, for automating git commands between different 2-level submodules.

## Git Helper Layout

The git deployer helper tools, can described as such:

    subt git [2-level].meta.checkout                # checkout a specific branch in 2-level meta
    subt git [2-level].meta.ignore                  # ignore all catkin files in 2-level meta
    subt git [2-level].meta.unignore                # ignore all catkin files in 2-level meta
    subt git [2-level].submodules.clean             # clean all uncommitted changes, in 2-level meta and 3-levels submodules
    subt git [2-level].submodules.pull              # pull the latest updates in 2-level meta (must be on a branch)
    subt git [2-level].submodules.reset             # reset 2-level meta to the commit associated with 1-level
    subt git [2-level].submodules.rm                # remove (deinitializes) the 2-level meta and all 3-level submodules

Using an example, `common`, the git deployer helpers tools are as such:

    subt git common.meta.checkout                   # checkout a specific branch in 2-level meta
    subt git common.meta.ignore                     # ignore all catkin files in 2-level meta
    subt git common.meta.unignore                   # ignore all catkin files in 2-level meta
    subt git common.submodules.clean                # clean all uncommitted changes, in 2-level meta and 3-levels submodules
    subt git common.submodules.pull                 # pull the latest updates in 2-level meta (must be on a branch)
    subt git common.submodules.reset                # reset 2-level meta to the commit associated with 1-level
    subt git common.submodules.rm                   # remove (deinitializes) the 2-level meta and all 3-level submodules

There is also a short-hand available for all 2-levels:

    subt git all.reset                              # resets all the submodules, to their `DETACHED HEAD` commit HASH as pushed on origin.
    subt git all.pull                               # pull the submodule's updates, when the submodules are checked-out at a specific branch.


## Git Helper Details

### Checkout

This will checkout a branch name in the 2-level repo (will create a branch if does not exist).

    # example, checkout a specific branch in the 2-level
    subt git common.meta.checkout -e branch=[branch name]

### Ignore

This will ignore all `.catkin_profile` files that git prompts to commit (but should not be committed).

    # example, reset all the submodules in common
    subt git common.meta.ignore

### Unignore

This will un-ignore all `.catkin_profile` files that git prompts to commit (if you want to commit).

    # example, reset all the submodules in common
    subt git common.meta.unignore

### Clean

This will clean all un-committed changes from 2-level and all 3-level submodules.

    # example, clean all the submodules in common
    subt git common.submodules.clean

    # some 2-levels have multiple options, for example ugv.ppc will clean all the submodules in ~/deploy_ws/src/ugv/planning-pc
    subt git ugv.ppc.submodule.clean

### Pull

This will pull the submodule HASH, when the 2-level is checked-out at a specific branch. So, if your 2-level branch is behind origin, it will pull the changes and pull the submodule changes.

    # example, reset all the submodules in common
    subt git common.submodules.pull

    # some 2-levels have multiple options, for example ugv.ppc will pull all the submodules in ~/deploy_ws/src/ugv/planning-pc
    subt git ugv.ppc.submodule.pull

### Reset

This will reset all the submodules, to their `DETACHED HEAD` commit HASH as pushed on origin.

    # example, reset all the submodules in common
    subt git common.submodules.reset

    # some 2-levels have multiple options, for example ugv.ppc will reset all the submodules in ~/deploy_ws/src/ugv/planning-pc
    subt git ugv.ppc.submodule.reset

Submodules reset as `DETACHED HEAD`. Make sure to **always checkout a branch** after its cloned.

The `reset`, will **RESET** your submodule to a HASH that was committed in the 1-level.

  - So, when you want to get all the submodules of what is pushed in the 1-level origin, use `reset`.
  - This is how we maintain the versioning, the HASH for all submodules are maintained by the 1-level, can be reset at any time.

### Rm (remove)

This will remove (deinitialize only) all the 3-level submodules, meaning they are just remove from your localhost, not on origin.

    # example, reset all the submodules in common
    subt git common.submodules.rm

    # some 2-levels have multiple options, for example ugv.ppc will remove all the submodules in ~/deploy_ws/src/ugv/planning-pc
    subt git ugv.ppc.submodule.rm
