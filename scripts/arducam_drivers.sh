# #!/bin/bash
# # This file is part of the jetson_stats package (https://github.com/rbonghi/jetson_stats or http://rnext.it).
# # Copyright (c) 2020 Raffaello Bonghi.
# #
# # This program is free software: you can redistribute it and/or modify
# # it under the terms of the GNU Affero General Public License as published by
# # the Free Software Foundation, either version 3 of the License, or
# # (at your option) any later version.
# #
# # This program is distributed in the hope that it will be useful,
# # but WITHOUT ANY WARRANTY; without even the implied warranty of
# # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# # GNU Affero General Public License for more details.
# #
# # You should have received a copy of the GNU Affero General Public License
# # along with this program. If not, see <http://www.gnu.org/licenses/>.


# # TODO Add enviroments variables:
# # - UID -> https://devtalk.nvidia.com/default/topic/996988/jetson-tk1/chip-uid/post/5100481/#5100481
# # - GCID, BOARD, EABI

# ###########################
# #### JETPACK DETECTION ####
# ###########################
# # Write version of jetpack installed
# # https://developer.nvidia.com/embedded/jetpack-archive
# jetson_jetpack()
# {
#     local JETSON_L4T=$1
#     local JETSON_JETPACK=""
#     case $JETSON_L4T in
#         "32.4.3") JETSON_JETPACK="4.4" ;;
#         "32.4.2") JETSON_JETPACK="4.4 DP" ;;
#         "32.3.1") JETSON_JETPACK="4.3" ;;
#         "32.2.3") JETSON_JETPACK="4.2.3" ;;
#         "32.2.1") JETSON_JETPACK="4.2.2" ;;
#         "32.2.0" | "32.2") JETSON_JETPACK="4.2.1" ;;
#         "32.1.0" | "32.1") JETSON_JETPACK="4.2" ;;
#         "31.1.0" | "31.1") JETSON_JETPACK="4.1.1" ;;
#         "31.0.2") JETSON_JETPACK="4.1" ;;
#         "31.0.1") JETSON_JETPACK="4.0" ;;
#         "28.2.1") JETSON_JETPACK="3.3 | 3.2.1" ;;
#         "28.2.0" | "28.2") JETSON_JETPACK="3.2" ;;
#         "28.1.0" | "28.1") JETSON_JETPACK="3.1" ;;
#         "27.1.0" | "27.1") JETSON_JETPACK="3.0" ;;
#         "24.2.1") JETSON_JETPACK="3.0 | 2.3.1" ;;
#         "24.2.0" | "24.2") JETSON_JETPACK="2.3" ;;
#         "24.1.0" | "24.1") JETSON_JETPACK="2.2.1 | 2.2" ;;
#         "23.2.0" | "23.2") JETSON_JETPACK="2.1" ;;
#         "23.1.0" | "23.1") JETSON_JETPACK="2.0" ;;
#         "21.5.0" | "21.5") JETSON_JETPACK="2.3.1 | 2.3" ;;
#         "21.4.0" | "21.4") JETSON_JETPACK="2.2 | 2.1 | 2.0 | 1.2 DP" ;;
#         "21.3.0" | "21.3") JETSON_JETPACK="1.1 DP" ;;
#         "21.2.0" | "21.2") JETSON_JETPACK="1.0 DP" ;;
#         *) JETSON_JETPACK="UNKNOWN" ;;
#     esac
#     # return type jetpack
#     echo $JETSON_JETPACK
# }

# ###########################
# ####  BOARD DETECTION  ####
# ###########################
# # Detection NVIDIA Jetson
# # Jetson TK1 can be detected with chip id
# # Reference:
# # 1. https://docs.nvidia.com/jetson/l4t/index.html (Devices Supported by This Document)
# # 2. https://github.com/rbonghi/jetson_stats/issues/48
# # 3. https://github.com/rbonghi/jetson_stats/issues/57
# jetson_board()
# {
#     local JETSON_CHIP_ID=$1
#     local JETSON_BOARD=$2
#     local JETSON_TYPE=""
#     # Detection jetson board
#     case $JETSON_BOARD in
#         *2180*) JETSON_TYPE="TX1" ;;
#         P3310*) JETSON_TYPE="TX2" ;;
#         P3489-0080*) JETSON_TYPE="TX2 4GB" ;;
#         P3489*) JETSON_TYPE="TX2i" ;;
#         P2888-0006*) JETSON_TYPE="AGX Xavier [8GB]" ;;
#         P2888-0001*|P2888-0004*) JETSON_TYPE="AGX Xavier [16GB]" ;;
#         P2888*) JETSON_TYPE="AGX Xavier [32GB]" ;;
#         P3448-0002*) JETSON_TYPE="Nano" ;;
#         P3448*) JETSON_TYPE="Nano (Developer Kit Version)" ;;
#         P3668-0001*) JETSON_TYPE="Xavier NX" ;;
#         P3668*) JETSON_TYPE="Xavier NX (Developer Kit Version)" ;;
#         *) JETSON_TYPE="" ;;
#     esac
#     # if JETSON_TYPE empty, check CHIP_ID for jetson TK1
#     if [ -z "$JETSON_TYPE" ] ; then
#         if [ "$JETSON_CHIP_ID" == "64" ] ; then
#             JETSON_TYPE="TK1"
#         fi
#     fi
#     # Return board name
#     echo $JETSON_TYPE
# }
# ###########################

# # Extract jetson chip id
# # JETSON_CHIP_ID=""
# # Because the /sys/module/tegra_fuse/parameters/tegra_chip_id file no longer exists in the JetPack5.0 system. Parameter passing error due to empty JETSON_CHIP_ID variable when calling jetson_board method. Setting a default value for JETSON_CHIP_ID ensures that the parameters are passed correctly. Modified date: 2022/6/27 13:26
# JETSON_CHIP_ID="UNKNOWN"
# if [ -f /sys/module/tegra_fuse/parameters/tegra_chip_id ]; then
#     JETSON_CHIP_ID=$(cat /sys/module/tegra_fuse/parameters/tegra_chip_id)
# fi
# # Ectract type board
# JETSON_SOC=""
# if [ -f /proc/device-tree/compatible ]; then
#     # Extract the last part of name
#     JETSON_SOC=$(tr -d '\0' < /proc/device-tree/compatible | sed -e 's/.*,//')
# fi
# # Extract boardids if exists
# JETSON_BOARDIDS=""
# if [ -f /proc/device-tree/nvidia,boardids ]; then
#     JETSON_BOARDIDS=$(tr -d '\0' < /proc/device-tree/nvidia,boardids)
# fi

# # Code name
# JETSON_CODENAME=""
# JETSON_TYPE="UNKNOWN"
# JETSON_MODULE="UNKNOWN"
# JETSON_BOARD="UNKNOWN"
# list_hw_boards()
# {
#     # Extract from DTS the name of the boards available
#     # Reference:
#     # 1. https://unix.stackexchange.com/questions/251013/bash-regex-capture-group
#     # 2. https://unix.stackexchange.com/questions/144298/delete-the-last-character-of-a-string-using-string-manipulation-in-shell-script
#     local s=$1
#     local regex='p([0-9-]+)' # Equivalent to p([\d-]*-)
#     while [[ $s =~ $regex ]]; do
#         local board_name=$(echo "P${BASH_REMATCH[1]::-1}")
#         #echo $board_name
#         # Load jetson type
#         JETSON_TYPE=$(jetson_board $JETSON_CHIP_ID $board_name)
#         # If jetson type is not empty the module name is the same
#         if [ ! -z "$JETSON_TYPE" ] ; then
#             JETSON_MODULE=$board_name
#             echo "JETSON_TYPE=\"$JETSON_TYPE\""
#             echo "JETSON_MODULE=\"$JETSON_MODULE\""
#         else
#             JETSON_BOARD=$board_name
#             echo "JETSON_BOARD=\"$JETSON_BOARD\""
#         fi
#         # Find next match
#         s=${s#*"${BASH_REMATCH[1]}"}
#     done
# }
# # Read DTS file
# # 1. https://devtalk.nvidia.com/default/topic/1071080/jetson-nano/best-way-to-check-which-tegra-board/
# # 2. https://devtalk.nvidia.com/default/topic/1014424/jetson-tx2/identifying-tx1-and-tx2-at-runtime/
# # 3. https://devtalk.nvidia.com/default/topic/996988/jetson-tk1/chip-uid/post/5100481/#5100481
# if [ -f /proc/device-tree/nvidia,dtsfilename ]; then
#     # ..<something>/hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-0000-p3449-0000-b00.dts
#     # The last third is the codename of the board
#     JETSON_CODENAME=$(tr -d '\0' < /proc/device-tree/nvidia,dtsfilename)
#     JETSON_CODENAME=$(echo ${JETSON_CODENAME#*"/hardware/nvidia/platform/"} | tr '/' '\n' | head -2 | tail -1 )
#     # The basename extract the board type
#     JETSON_DTS="$(tr -d '\0' < /proc/device-tree/nvidia,dtsfilename | sed 's/.*\///')"
#     # List of all boards
#     eval $(list_hw_boards "$JETSON_DTS")
# fi

# # Print description
# JETSON_MACHINE=""
# if [ ! -z "$JETSON_TYPE" ] ; then
#     JETSON_MACHINE="NVIDIA Jetson $JETSON_TYPE"
# fi
# # Export variables
# export JETSON_CHIP_ID
# export JETSON_SOC
# export JETSON_BOARDIDS
# export JETSON_CODENAME
# export JETSON_MODULE
# export JETSON_BOARD
# export JETSON_TYPE
# export JETSON_MACHINE

# # Write CUDA architecture
# # https://developer.nvidia.com/cuda-gpus
# # https://devtalk.nvidia.com/default/topic/988317/jetson-tx1/what-should-be-the-value-of-cuda_arch_bin/
# case $JETSON_TYPE in
#     *Xavier*) JETSON_CUDA_ARCH_BIN="7.2" ;;
#     TX2*) JETSON_CUDA_ARCH_BIN="6.2" ;;
#     TX1* | Nano*) JETSON_CUDA_ARCH_BIN="5.3" ;;
#     TK1) JETSON_CUDA_ARCH_BIN="3.2" ;;
#     * ) JETSON_CUDA_ARCH_BIN="NONE" ;;
# esac
# # Export Jetson CUDA ARCHITECTURE
# export JETSON_CUDA_ARCH_BIN

# # Serial number
# # https://devtalk.nvidia.com/default/topic/1055507/jetson-nano/nano-serial-number-/
# JETSON_SERIAL_NUMBER=""
# if [ -f /sys/firmware/devicetree/base/serial-number ]; then
#     JETSON_SERIAL_NUMBER=$(tr -d '\0' </sys/firmware/devicetree/base/serial-number)
# fi
# # Export NVIDIA Serial Number
# export JETSON_SERIAL_NUMBER

# # NVIDIA Jetson version
# # reference https://devtalk.nvidia.com/default/topic/860092/jetson-tk1/how-do-i-know-what-version-of-l4t-my-jetson-tk1-is-running-/
# # https://stackoverflow.com/questions/16817646/extract-version-number-from-a-string
# # https://askubuntu.com/questions/319307/reliably-check-if-a-package-is-installed-or-not
# # https://github.com/dusty-nv/jetson-inference/blob/7e81381a96c1ac5f57f1728afbfdec7f1bfeffc2/tools/install-pytorch.sh#L296
# if [ -f /etc/nv_tegra_release ]; then
#     # L4T string
#     # First line on /etc/nv_tegra_release 
#     # - "# R28 (release), REVISION: 2.1, GCID: 11272647, BOARD: t186ref, EABI: aarch64, DATE: Thu May 17 07:29:06 UTC 2018"
#     JETSON_L4T_STRING=$(head -n 1 /etc/nv_tegra_release)
#     # Load release and revision
#     JETSON_L4T_RELEASE=$(echo $JETSON_L4T_STRING | cut -f 2 -d ' ' | grep -Po '(?<=R)[^;]+')
#     JETSON_L4T_REVISION=$(echo $JETSON_L4T_STRING | cut -f 2 -d ',' | grep -Po '(?<=REVISION: )[^;]+')
# elif dpkg --get-selections | grep -q "^nvidia-l4t-core[[:space:]]*install$"; then
#     # Read version
#     JETSON_L4T_STRING=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-core)
#     # extract version
#     JETSON_L4T_ARRAY=$(echo $JETSON_L4T_STRING | cut -f 1 -d '-')
#     # Load release and revision
#     JETSON_L4T_RELEASE=$(echo $JETSON_L4T_ARRAY | cut -f 1 -d '.')
#     JETSON_L4T_REVISION=${JETSON_L4T_ARRAY#"$JETSON_L4T_RELEASE."}
# else
#     # Load release and revision
#     JETSON_L4T_RELEASE="N"
#     JETSON_L4T_REVISION="N.N"
# fi
# # Write Jetson description
# JETSON_L4T="$JETSON_L4T_RELEASE.$JETSON_L4T_REVISION"
# # Export Release L4T
# export JETSON_L4T_RELEASE
# export JETSON_L4T_REVISION
# export JETSON_L4T

# JETSON_JETPACK=$(jetson_jetpack $JETSON_L4T)
# # Export Jetson Jetpack installed
# export JETSON_JETPACK
# # EOF
JETSON_TYPE=$(cat /sys/firmware/devicetree/base/model)
echo $JETSON_TYPE

content=$(dpkg-query --showformat='${Version}' --show nvidia-l4t-kernel)
CONFIG_FILE_NAME=modules.txt
CONFIG_FILE_DOWNLOAD_LINK=https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/modules.txt
RED='\033[0;31m'
NC='\033[0m' # No Color

updateModules()
{
    rm -f $CONFIG_FILE_NAME
    wget -O $CONFIG_FILE_NAME $CONFIG_FILE_DOWNLOAD_LINK
    source $CONFIG_FILE_NAME
}

listModules()
{
    if [ ! -f $CONFIG_FILE_NAME ]; then
        updateModules
    fi
    source $CONFIG_FILE_NAME
    echo "Supported modules:"
    for key in ${!module_cfg_names[*]};do
    echo -e "\t$key"
    done
    echo ""
}

helpFunction()
{
    if [ ! -f $CONFIG_FILE_NAME ]; then
        updateModules
    fi
    echo ""
    echo "Usage: $0 [option]... -m <moduel name>"
    echo -e "Options:"
    echo -e "\t-m <module name>\tSpecify the module name."
    echo -e "\t-h \t\t\tShow this information."
    echo -e "\t-l \t\t\tUpdate and list available modules."
    echo ""
    listModules
    exit 1
}

while getopts hlm: flag
do
    case "${flag}" in
        m)  module=${OPTARG};;
        l)  updateModules
            listModules
            exit 1
            ;;
        ?)  helpFunction;;
    esac
done

if [ ! -f $CONFIG_FILE_NAME ]; then
    updateModules
fi

source $CONFIG_FILE_NAME

if [ -z $module ]; then
    helpFunction
fi

module_cfg_name=${module_cfg_names[$module]}
module_cfg_download_link=${module_cfg_download_links[$module]}

if [[ (-z $module_cfg_name) || (-z $module_cfg_download_link) ]]; then
    echo -e "${RED}Unsupported modules.${NC}"
    echo ""
    listModules
    exit -1
fi

# echo "module_cfg_name: $module_cfg_name"
# echo "module_cfg_download_link: $module_cfg_download_link"

rm -f $module_cfg_name
wget -O $module_cfg_name $module_cfg_download_link
source $module_cfg_name

download_link=
pkg_name=
if [[ $JETSON_TYPE == *"Xavier NX"* ]]; then
	download_link=${nx_download_links[$content]}
	pkg_name=${nx_names[$content]}
fi

if [[ $JETSON_TYPE == *"Orin NX"* ]]; then
	download_link=${orin_nx_download_links[$content]}
	pkg_name=${orin_nx_names[$content]}
fi

if [[ $JETSON_TYPE == *"Nano"* ]]; then
	download_link=${nano_download_links[$content]}
	pkg_name=${nano_names[$content]}
fi

if [[ (-z $pkg_name) || (-z $download_link) ]]; then
    echo -e "${RED}"
	echo -e "Cannot find the corresponding deb package, please send the following information to support@arducam.com"
	echo -e "Kernel version: " $content
	echo -e "Jetson type: " $JETSON_TYPE
    echo -e "${NC}"
	exit -1
fi

rm -rf $pkg_name
 
wget $download_link

if [[ ( $? -ne 0) || (! -f "${pkg_name}") ]]; then
	echo -e "${RED}download failed${NC}"
	exit -1
fi

if [ $content == "4.9.140-tegra-32.4.3-20200924161919" ]; then
	if [[ $JETSON_TYPE == *"Xavier NX"* ]]; then
		unzip 20200924161919.zip && sudo dpkg -i 20200924161919/XavierNx/arducam-nvidia-l4t-kernel_4.9.140-32.4.3-20201021145043_arm64.deb
	fi

	if [[ $JETSON_TYPE == *"Nano"* ]]; then
		unzip 20200924161919.zip && sudo dpkg -i 20200924161919/Nano/arducam-nvidia-l4t-kernel_4.9.140-32.4.3-20201021145043_arm64.deb
	fi
else
	sudo dpkg -i $pkg_name
fi

if [ $? -ne 0 ]; then
    echo ""
	echo -e "${RED}Unknown error, please send the error message to support@arducam.com${NC}"
	exit -1
fi

echo ""
echo "reboot now?(y/n):"
read USER_INPUT
case $USER_INPUT in
'y'|'Y')
    echo "reboot"
    sudo reboot
;;
*)
    echo "cancel"
;;
esac
