#!/bin/bash

# The dependencies (glog, protobuf and websockets) are built based on the CMake options. 
# Default they will be cloned and build in <ToFrootdir>/libaditof 

source_dir=$(cd "$(dirname "$0")/../.."; pwd)



print_help() {
        echo "./setup [OPTIONS]"
        echo ""
        echo "-h|--help"
        echo "        Print a usage message briefly summarizing the command line options available, then exit."
        echo "-y|--yes"
        echo "        Automatic yes to prompts."
        echo "-b|--buildir"
        echo "        Specify the build directory of the SDK."
        echo "-j|--jobs"
        echo "        Specify the number of jobs to run in parallel when building dependencies and the SDK"
        echo ""
}

install_required_packages() {
        sudo apt install -y build-essential cmake python3-dev \
        libssl-dev git
}

yes_or_exit() {
        message=$1
        while true; do
                read -p "${message} [Y/n]" yn
                case $yn in
                        [Yy]* ) break;;
                        [Nn]* ) exit;;
                        * ) echo "Please answer yes or no.";;
                esac
        done
}

setup() {
        NUM_JOBS=$(nproc --all)

        while [[ $# -gt 0 ]]
        do
        key="$1"
        case $key in
                -y|--yes)
                answer_yes="True"
                shift # next argument
                ;;
                -h|--help)
                display_help="True"
                shift # next argument
                ;;
                -b|--builddir)
                build_dir=$2
                shift # past argument
                shift # past value
                ;;
                -j|--jobs)
                NUM_JOBS=$2
                shift # next argument
                shift # next value
                ;;
                *)    # unknown option
                POSITIONAL+=("$1") # save it in an array for later
                shift # past argument
                ;;
        esac
        done
        set -- "${POSITIONAL[@]}" # restore positional parameters

        if [[ "${display_help}" == "True" ]]; then
                print_help
                exit
        fi

        install_required_packages

        source_dir=$(cd "$(dirname "$0")/../.."; pwd)

        if [[ -z "${build_dir}" ]]; then
                build_dir=$(pwd)/build
        fi

        echo "The sdk will be built in: ${build_dir}"

        if [[ -z "${answer_yes}" ]]; then
             yes_or_exit "Do you want to continue?"
        fi

        mkdir -p "${build_dir}"
        

        if [[ -f ../../../libs/libtofi_compute.so && -f ../../../libs/libtofi_config.so ]]; then
                CMAKE_OPTIONS="-DNXP=1 -DWITH_PYTHON=1 -DCMAKE_BUILD_TYPE=Release"
        else
                CMAKE_OPTIONS="-DNXP=1 -DUSE_DEPTH_COMPUTE_OPENSOURCE=ON -DWITH_PYTHON=1 -DCMAKE_BUILD_TYPE=Release"
        fi

        tof_dir=$(pwd)/../../
        
        
        pushd "${build_dir}"
        cmake "${source_dir}" ${CMAKE_OPTIONS} 
        make -j ${NUM_JOBS}
}

setup $@
