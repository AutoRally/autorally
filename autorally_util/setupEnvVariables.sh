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
        if [ $AR_CHASSIS_SERIAL == '75439313737351F052A0' ]
          then
            export AR_CHASSIS="gamma"
        fi
fi
if [[ $devList == *"arServoController"* ]] # If Pololu Servo Controller is connected...
    then
        AR_CHASSIS_SERIAL=`udevadm info --query=property --name=/dev/arServoController | grep 'ID_SERIAL_SHORT'`
        AR_CHASSIS_SERIAL=${AR_CHASSIS_SERIAL#*=}
        if [ $AR_CHASSIS_SERIAL == '00082495' ]
          then
            export AR_CHASSIS="beta"
        elif [ $AR_CHASSIS_SERIAL == '00026587' ]
          then
            export AR_CHASSIS="alpha"
        fi
fi
if [[ $DEBUG == true ]]; then
    echo "AR_CHASSIS set to ${AR_CHASSIS}"
fi

# Setup cameras
if [[ $devList == *"arCamera_b09d0100d70a0a"* ]]
    then
        export AR_RIGHTCAM="b09d0100d70a0a"
elif [[ $devList == *"arCamera_b09d0100d70a17"* ]]
    then
        export AR_RIGHTCAM="b09d0100d70a17"
fi
if [[ $DEBUG == true ]]; then
    echo "AR_RIGHTCAM set to ${AR_RIGHTCAM}"
fi

if [[ $devList == *"arCamera_b09d0100d70a1c"* ]]
    then
        export AR_LEFTCAM="b09d0100d70a1c"
elif [[ $devList == *"arCamera_b09d0100c6fd73"* ]]
    then
        export AR_LEFTCAM="b09d0100c6fd73"
fi
if [[ $DEBUG == true ]]; then
    echo "AR_LEFTCAM set to ${AR_LEFTCAM}"
fi

