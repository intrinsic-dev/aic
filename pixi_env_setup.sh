#!/usr/bin/bash
set -e

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-transport/shared_memory/enabled=false}"

# torchcodec 0.10 needs libtorch.so + libc10.so + libavutil.so.60 on the dlopen
# search path. The conda env has these, but torch.ops.load_library uses
# ctypes.CDLL which doesn't automatically search torch's lib subdir.
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    _torchlib="${CONDA_PREFIX}/lib/python3.12/site-packages/torch/lib"
    if [[ -d "$_torchlib" ]]; then
        export LD_LIBRARY_PATH="${_torchlib}:${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
    unset _torchlib
fi
