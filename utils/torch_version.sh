#!/bin/bash -e

# REF: https://github.com/dusty-nv/jetson-containers/blob/master/scripts/docker_build_ml.sh

PYTORCH_URL=''
PYTORCH_WHL=''
PYTORCH_VERSION=''
TORCH_VISION_VERSION=''



if [[ $L4T_RELEASE -eq 32 ]]; then   # JetPack 4.x
		
	# PyTorch v1.9.0
	PYTORCH_URL="https://nvidia.box.com/shared/static/h1z9sw4bb1ybi0rm3tu8qdj8hs05ljbm.whl"
	PYTORCH_WHL="torch-1.9.0-cp36-cp36m-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.10.0"
				
	# PyTorch v1.10.0
	PYTORCH_URL="https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl"
	PYTORCH_WHL="torch-1.10.0-cp36-cp36m-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.11.1"
				
	# PyTorch v1.11.0
	PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v461/pytorch/torch-1.11.0a0+17540c5-cp36-cp36m-linux_aarch64.whl"
	PYTORCH_WHL="torch-1.11.0a0+17540c5-cp36-cp36m-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.11.3"
				
elif [[ $L4T_RELEASE -eq 34 ]]; then   # JetPack 5.0.0 (DP) / 5.0.1 (DP2)

	# PyTorch v1.11.0
	PYTORCH_URL="https://nvidia.box.com/shared/static/ssf2v7pf5i245fk4i0q926hy4imzs2ph.whl"
	PYTORCH_WHL="torch-1.11.0-cp38-cp38-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.12.0"
				
	# PyTorch v1.12.0
	PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl"
	PYTORCH_WHL="torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.12.0"
				
elif [[ $L4T_RELEASE -eq 35 ]] && [[ $L4T_REVISION_MAJOR -le 1 ]]; then   # JetPack 5.0.2 (GA)

	# PyTorch v1.11.0
	PYTORCH_URL="https://nvidia.box.com/shared/static/ssf2v7pf5i245fk4i0q926hy4imzs2ph.whl"
	PYTORCH_WHL="torch-1.11.0-cp38-cp38-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.12.0"
				
	# PyTorch v1.12.0
	
				
	# PyTorch v1.13.0
	PYTORCH_URL="https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-1.13.0a0+340c4120.nv22.06-cp38-cp38-linux_aarch64.whl"
	PYTORCH_WHL="torch-1.13.0a0+340c4120.nv22.06-cp38-cp38-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.13.0"
				
elif [[ $L4T_RELEASE -eq 35 ]]; then   # JetPack 5.1.x

	# PyTorch v2.0 (JetPack 5.1 / L4T R35.2.1)
	#build_pytorch "https://nvidia.box.com/shared/static/rehpfc4dwsxuhpv4jgqv8u6rzpgb64bq.whl" \
	#			"torch-2.0.0a0+ec3941ad.nv23.2-cp38-cp38-linux_aarch64.whl" \
	#			"l4t-pytorch:r$L4T_VERSION-pth2.0-py3" \
	#			"v0.14.1" \
	#			"v0.13.1"
	
	# PyTorch v2.0 (JetPack 5.1.1 / L4T R35.3.1)
	PYTORCH_URL="https://nvidia.box.com/shared/static/i8pukc49h3lhak4kkn67tg9j4goqm0m7.whl"
	PYTORCH_WHL="torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
	TORCH_VISION_VERSION="v0.15.1"

else
	echo "warning -- unsupported L4T R$L4T_VERSION, skipping PyTorch..."
fi
