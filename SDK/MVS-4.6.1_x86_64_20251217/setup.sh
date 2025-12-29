#!/bin/bash

DIRNAME=`dirname $0`
#PWD = `pwd`

cd $DIRNAME
source ./ExportAppName.sh

if [ -z "${MVAppName}" ]; then
   echo "please use ./setup.sh"
   return
fi


#sed -i "s/export LD_LIBRARY_PATH/#export LD_LIBRARY_PATH/g" ~/.bashrc
source ~/.bashrc
if [ ! -d "/opt/${MVAppName}" ]; then
	echo "Install ${MVAppName},Please wait..."
	mkdir -p /opt/${MVAppName}
	if [ -f "./${MVAppName}.tar.gz" ]; then
		tar -C /opt/${MVAppName} -xzf ./${MVAppName}.tar.gz
	elif [ -d "./${MVAppName}" ]; then
		cp -r ./${MVAppName}/* /opt/${MVAppName}/
	else
		echo "Error: Neither ${MVAppName}.tar.gz nor ${MVAppName}/ directory found!"
		exit 1
	fi
else
	echo "Uninstall ${MVAppName},Please wait..."
	rm -rf /opt/${MVAppName}
	echo "Install ${MVAppName},Please wait..."
	mkdir -p /opt/${MVAppName}
	if [ -f "./${MVAppName}.tar.gz" ]; then
		tar -C /opt/${MVAppName} -xzf ./${MVAppName}.tar.gz
	elif [ -d "./${MVAppName}" ]; then
		cp -r ./${MVAppName}/* /opt/${MVAppName}/
	else
		echo "Error: Neither ${MVAppName}.tar.gz nor ${MVAppName}/ directory found!"
		exit 1
	fi
fi
#
if [ ! -d "/usr/local/Qt-5.6.3/lib/fonts" ]; then
	mkdir -p /usr/local/Qt-5.6.3/lib/fonts
	if [ -d "/opt/${MVAppName}/bin/fonts" ]; then
		cp -r /opt/${MVAppName}/bin/fonts/* /usr/local/Qt-5.6.3/lib/fonts
	elif [ -d "/opt/${MVAppName}/bin/Fonts" ]; then
		cp -r /opt/${MVAppName}/bin/Fonts/* /usr/local/Qt-5.6.3/lib/fonts
	fi
else
	echo "path exist..."
fi
#

echo "Set up the SDK environment..."

bash $DIRNAME/set_usb_priority.sh
bash $DIRNAME/set_virtualserial_priority.sh
source $DIRNAME/set_env_path.sh /opt/${MVAppName}
source $DIRNAME/set_sdk_version.sh

if [ -d /opt/${MVAppName}/driver/gige ]; then
	cd /opt/${MVAppName}/driver/gige
	if [ -f /opt/${MVAppName}/driver/gige/unload.sh ]; then
		./unload.sh
	fi
	if [ -f /opt/${MVAppName}/driver/gige/driver_self_starting.sh ]; then
		/opt/${MVAppName}/driver/gige/driver_self_starting.sh 1 
	fi
fi
if [ -d /opt/${MVAppName}/bin ]; then
	cd /opt/${MVAppName}/bin
	if [ -f /opt/${MVAppName}/bin/script_self_starting.sh ]; then
		/opt/${MVAppName}/bin/script_self_starting.sh 1 
	fi
fi

if [ -d /opt/${MVAppName}/logserver ]; then
	cd /opt/${MVAppName}/logserver
	if [ -f ./RemoveServer.sh ]; then
		./RemoveServer.sh
	fi
	if [ -f ./InstallServer.sh ]; then
		./InstallServer.sh
	fi
fi

#����MVFG����������־����
if [ -d /opt/${MVAppName}/driver/pcie ]; then
	cd /opt/${MVAppName}/driver/pcie
	if [ -f /opt/${MVAppName}/driver/pcie/unload.sh ]; then
		bash ./unload.sh
	fi
	if [ -f /opt/${MVAppName}/driver/pcie/driver_self_starting.sh ]; then
		/opt/${MVAppName}/driver/pcie/driver_self_starting.sh 1 
	fi
fi



echo "Install ${MVAppName} complete!"
echo "Tips: You should be launch a new terminal or execute source command for the bash environment!"
cd $PWD



