DEBUG=false
while [[ $# > 0 ]]
do
key="$1"

case $key in
    -d|--debug)
    DEBUG=true
    shift
    ;;
esac
done

if [[ $MASTER_HOSTNAME == "localhost" ]]
    then
        devList=$(ls /dev/)
else
        devList=$(ssh $MASTER_HOSTNAME ls /dev/)
fi

# Locate config folder
export AR_CONFIG_PATH=`rospack find autorally_util`/config

# Setup servo controller and determine chassis
if [[ $devList == *"arChassis"* ]] # If Arduino Due is connected...
    then
        AR_CHASSIS_SERIAL=`udevadm info --query=property --name=/dev/arChassis | grep 'ID_SERIAL_SHORT'`
        AR_CHASSIS_SERIAL=${AR_CHASSIS_SERIAL#*=}
        if [ $AR_CHASSIS_SERIAL == 'ARDUINO_DUE_SERIAL_NUMBER' ]
          then
            export AR_CHASSIS="CHASSIS_NAME"
        #elif [ $AR_CHASSIS_SERIAL == 'ARDUINO_DUE_SERIAL_NUMBER2' ]
        #  then
        #    export AR_CHASSIS="CHASSIS_NAME2"
        fi
fi
if [[ $DEBUG == true ]]; then
    echo "AR_CHASSIS set to ${AR_CHASSIS}"
fi

# Setup cameras

# Right Camera
export AR_RIGHTCAM_CONNECTED=true
if [[ $devList == *"arCamera_GUID"* ]]
    then
        export AR_RIGHTCAM="GUID"
#elif [[ $devList == *"arCamera_GUID"* ]]
#    then
#        export AR_RIGHTCAM="GUID"
else
    export AR_RIGHTCAM_CONNECTED=false
fi
if [[ $DEBUG == true ]]; then
    echo "AR_RIGHTCAM set to ${AR_RIGHTCAM}"
fi

# Left Camera
export AR_LEFTCAM_CONNECTED=true
if [[ $devList == *"arCamera_GUID"* ]]
    then
        export AR_LEFTCAM="GUID"
#elif [[ $devList == *"arCamera_GUID"* ]]
#    then
#        export AR_LEFTCAM="GUID"
else
    export AR_LEFTCAM_CONNECTED=false
fi
if [[ $DEBUG == true ]]; then
    echo "AR_LEFTCAM set to ${AR_LEFTCAM}"
fi

