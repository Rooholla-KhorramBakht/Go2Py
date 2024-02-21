#!/bin/bash

# Note: This shell script is inspired by the Orbit simulator: https://isaac-B1Py.github.io/
#==
# Configurations
#==

# Exits if error occurs
set -e

# Set tab-spaces
tabs 4

# get source directory
export GO2PY_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

#==
# Helper functions
#==

# extract the python from isaacsim
extract_python_exe() {
    # Check if IsaacSim directory manually specified
    # Note: for manually build isaacsim, this: _build/linux-x86_64/release
    if [ ! -z ${ISAACSIM_PATH} ];
    then
        # Use local build
        build_path=${ISAACSIM_PATH}
    else
        # Use TeamCity build
        build_path=${GO2PY_PATH}/_isaac_sim
    fi
    # check if using conda
    if ! [[ -z "${CONDA_PREFIX}" ]]; then
        # use conda python
        local python_exe=${CONDA_PREFIX}/bin/python
    else
        # use python from kit
        local python_exe=${build_path}/python.sh
    fi
    # check if there is a python path available
    if [ ! -f "${python_exe}" ]; then
        echo "[ERROR] No python executable found at path: ${build_path}" >&2
        exit 1
    fi
    # return the result
    echo ${python_exe}
}

# check if input directory is a python extension and install the module
install_B1Py_extension() {
    # retrieve the python executable
    python_exe=$(extract_python_exe)
    # if the directory contains setup.py then install the python module
    if [ -f "$1/setup.py" ];
    then
        echo -e "\t module: $1"
        ${python_exe} -m pip install --editable $1
    fi
}

# update the vscode settings from template and isaac sim settings
update_vscode_settings() {
    echo "[INFO] Setting up vscode settings..."
    # retrieve the python executable
    python_exe=$(extract_python_exe)
    # run the setup script
    ${python_exe} ${GO2PY_PATH}/.vscode/tools/setup_vscode.py
}

# print the usage description
print_help () {
    echo -e "\nusage: $(basename "$0") [-h] [-i] [-e] [-f] [-p] [-s] [-v] [-d] [-c] -- Utility to manage extensions in Orbit."
    echo -e "\noptional arguments:"
    echo -e "\t-h, --help           Display the help content."
    echo -e "\t-i, --install        Install the extensions inside Isaac B1Py."
    echo -e "\t-f, --format         Run pre-commit to format the code and check lints."
    echo -e "\t-p, --python         Run the python executable (python.sh) provided by Isaac Sim."
    # echo -e "\t-s, --sim            Run the simulator executable (isaac-sim.sh) provided by Isaac Sim."
    echo -e "\t-s, --sim            Run the B1Py Isaac Sim simulation node."
    echo -e "\t-v, --vscode         Generate the VSCode settings file from template."
    echo -e "\n" >&2
}

#==
# Main
#==

# check argument provided
if [ -z "$*" ]; then
    echo "[Error] No arguments provided." >&2;
    print_help
    exit 1
fi

# pass the arguments
while [[ $# -gt 0 ]]; do
    # read the key
    case "$1" in
        -i|--install)
            # install the python packages for supported reinforcement learning frameworks
            echo "[INFO] Installing extra requirements such as learning frameworks..."
            python_exe=$(extract_python_exe)
            # install the rl-frameworks specified
            ${python_exe} -m pip install -e ${GO2PY_PATH}
            shift # past argument
            ;;
        -f|--format)
            # reset the python path to avoid conflicts with pre-commit
            # this is needed because the pre-commit hooks are installed in a separate virtual environment
            # and it uses the system python to run the hooks
            if [ -n "${CONDA_DEFAULT_ENV}" ]; then
                cache_pythonpath=${PYTHONPATH}
                export PYTHONPATH=""
            fi
            # run the formatter over the repository
            # check if pre-commit is installed
            if ! command -v pre-commit &>/dev/null; then
                echo "[INFO] Installing pre-commit..."
                pip install pre-commit
            fi
            # always execute inside the Orbit directory
            echo "[INFO] Formatting the repository..."
            cd ${GO2PY_PATH}
            pre-commit run --all-files
            cd - > /dev/null
            # set the python path back to the original value
            if [ -n "${CONDA_DEFAULT_ENV}" ]; then
                export PYTHONPATH=${cache_pythonpath}
            fi
            shift # past argument
            # exit neatly
            break
            ;;
        -p|--python)
            # run the python provided by isaacsim
            python_exe=$(extract_python_exe)
            echo "[INFO] Using python from: ${python_exe}"
            shift # past argument
            ${python_exe} $@
            # exit neatly
            break
            ;;
        # -s|--sim)
        #     # run the simulator exe provided by isaacsim
        #     isaacsim_exe=$(extract_isaacsim_exe)
        #     echo "[INFO] Running isaac-sim from: ${isaacsim_exe}"
        #     shift # past argument
        #     ${isaacsim_exe} --ext-folder ${GO2PY_PATH}/source/extensions $@
        #     # exit neatly
        #     break
        #     ;;

        -s|--sim)
            # run the simulator exe provided by isaacsim
            python_exe=$(extract_python_exe)
            # install the rl-frameworks specified
            ${python_exe} ${GO2PY_PATH}/B1Py/sim/isaac/isaacsim_node.py 
            # exit neatly
            break
            ;;
            
        -v|--vscode)
            # update the vscode settings
            update_vscode_settings
            shift # past argument
            # exit neatly
            break
            ;;
        -h|--help)
            print_help
            exit 1
            ;;
        *) # unknown option
            echo "[Error] Invalid argument provided: $1"
            print_help
            exit 1
            ;;
    esac
done