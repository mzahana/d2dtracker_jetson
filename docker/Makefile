# Makefile

UNAME="admin"
U_UID=1000
U_GID=1000
.PHONY: d2dtracker-jetson openvins-jetson

all: d2dtracker-jetson openvins-jetson


d2dtracker-jetson:
	docker build --build-arg USER_GID=$(U_GID) --build-arg USER_UID=$(U_UID) --build-arg USERNAME=$(UNAME) --build-arg BASE_IMAGE="dustynv/ros:humble-pytorch-l4t-r$(L4TVER)" -t mzahana/d2dtracker-jetson:r$(L4TVER) . -f Dockerfile.d2dtracker
openvins-jetson:
	docker build --build-arg USER_GID=$(U_GID) --build-arg USER_UID=$(U_UID) --build-arg USERNAME=$(UNAME) --build-arg BASE_IMAGE="dustynv/ros:humble-pytorch-l4t-r$(L4TVER)" -t mzahana/openvins-jetson:r$(L4TVER) . -f Dockerfile.openvins

# d2dtracker-jetson-r35.1.0:
# 	docker build --build-arg OPENCV_DEB=$(CV_DEB) --build-arg OPENCV_URL=$(CV_URL) --build-arg USER_GID=$(U_GID) --build-arg USER_UID=$(U_UID) --build-arg USERNAME=$(UNAME) --build-arg BASE_IMAGE="dustynv/ros:humble-pytorch-l4t-r$(L4TVER)" -t mzahana/d2dtracker:r35.1.0 . -f Dockerfile.d2dtracker.3