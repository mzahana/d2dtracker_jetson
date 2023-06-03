# Makefile

UNAME="admin"
U_UID=1000
U_GID=1000
.PHONY: d2dtracker-jetson

all: d2dtracker-jetson


d2dtracker-jetson:
	docker build --build-arg USER_GID=$(U_GID) --build-arg USER_UID=$(U_UID) --build-arg USERNAME=$(UNAME) --build-arg BASE_IMAGE="dustynv/ros:humble-pytorch-l4t-r$(L4TVER)" -t mzahana/d2dtracker-jetson:r$(L4TVER) . -f Dockerfile.d2dtracker.2
