#!/usr/bin/env bash
eval "$(cat $(dirname "${BASH_SOURCE[0]}")/header.sh)"

if chk_flag --help $@; then
    title "$__file_name [ args ]: simply runs terraform plan in the azurebooks/subt directory, args are passed to terraform."
    terraform plan $@
    exit 0
fi

cd $__dir/../subt

terraform plan $@

cd $__call_dir
