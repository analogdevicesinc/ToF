#!/bin/bash

# This script downloads and installs the dependencies (glog, protobuf and websockets) and builds the sdk.

source_dir=$(cd "$(dirname "$0")/../.."; pwd)

source ./utils.sh

print_help() {
        echo "./setup [OPTIONS]"
        echo ""
        echo "-h|--help"
        echo "        Print a usage message briefly summarizing the command line options available, then exit."
        echo "-y|--yes"
        echo "        Automatic yes to prompts."
        echo "-b|--buildir"
        echo "        Specify the build directory of the SDK."
        echo "-d|--depsdir"
        echo "        Specify the directory where the dependencies will be downloaded."
        echo "-i|--depsinstalldir"
        echo "        Specify the directory where the dependencies will be installed."
        echo "-j|--jobs"
        echo "        Specify the number of jobs to run in parallel when building dependencies and the SDK"
        echo ""
}

install_required_packages() {
        sudo apt install -y build-essential cmake python-dev python3-dev \
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
                -d|--depsdir)
                deps_dir=$2
                shift # past argument
                shift # past value
                ;;
                -i|--depsinstalldir)
                deps_install_dir=$2
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

        if [[ -z "${deps_dir}" ]]; then
                deps_dir=$(pwd)/deps
        fi

        echo "The deps will be downloaded in: ${deps_dir}"

        if [[ -z "${deps_install_dir}" ]]; then
                deps_install_dir=${deps_dir}/installed
        fi

        echo "The deps will be installed in: ${deps_install_dir}"

        if [[ -z "${answer_yes}" ]]; then
             yes_or_exit "Do you want to continue?"
        fi

        mkdir -p "${build_dir}"
        mkdir -p "${deps_dir}"
        mkdir -p "${deps_install_dir}"

        pushd "${deps_install_dir}"
        deps_install_dir=$(pwd)
        popd

        get_deps_source_code ${deps_dir}

        build_and_install_glog ${deps_dir}/glog ${deps_install_dir}/glog
        build_and_install_protobuf ${deps_dir}/protobuf ${deps_install_dir}/protobuf
        build_and_install_websockets ${deps_dir}/libwebsockets ${deps_install_dir}/websockets

        CMAKE_OPTIONS="-DUSE_ITOF=1 -DNXP=1"
        PREFIX_PATH="${deps_install_dir}/glog;${deps_install_dir}/protobuf;${deps_install_dir}/websockets;"

        pushd "${build_dir}"
        cmake "${source_dir}" ${CMAKE_OPTIONS} -DCMAKE_PREFIX_PATH="${PREFIX_PATH}"
        make -j ${NUM_JOBS}
}

setup $@
