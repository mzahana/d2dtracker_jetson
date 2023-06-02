.PHONY: d2dtracker-jetson

all: d2dtracker-jetson


d2dtracker-jetson:
	docker build --build-arg BASE_IMAGE=dustynv/ros:humble-pytorch-l4t-r${L4T_VERSION} -t mzahana/d2dtracker-jetson:r${L4T_VERSION} . -f Dockerfile.d2dtracker.2
