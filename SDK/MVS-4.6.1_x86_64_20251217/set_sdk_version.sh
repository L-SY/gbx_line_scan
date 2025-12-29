#!/bin/bash

source /etc/profile
MV_CAM_RUNENV_PATH=$MVCAM_COMMON_RUNENV
if [ -z "${MV_CAM_RUNENV_PATH}" ]; then
	echo "Warning: MVCAM_COMMON_RUNENV is not set, skipping library setup"
	exit 0
fi
if [ ! -d "${MV_CAM_RUNENV_PATH}" ]; then
	echo "Warning: ${MV_CAM_RUNENV_PATH} does not exist, skipping library setup"
	exit 0
fi
cd  ${MV_CAM_RUNENV_PATH} || exit 0

echo "create link to dynamic library"
if [ -d "${MV_CAM_RUNENV_PATH}/64" ]; then
ldconfig -n ./64
fi

if [ -d "${MV_CAM_RUNENV_PATH}/32" ]; then
ldconfig -n ./32
fi

if [ -d "${MV_CAM_RUNENV_PATH}/armhf" ]; then
ldconfig -n ./armhf
fi

if [ -d "${MV_CAM_RUNENV_PATH}/aarch64" ]; then
ldconfig -n ./aarch64
fi

if [ -d "${MV_CAM_RUNENV_PATH}/arm-none" ]; then
ldconfig -n ./arm-none
fi
