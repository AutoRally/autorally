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

# The following function was pulled from
# http://stackoverflow.com/a/8574392
containsElement () {
  local e
  for e in "${@:2}"; do [[ "$e" == "$1" ]] && return 0; done
  return 1
}

# Get list of devices connected to computer
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

## Add camera serial numbers to these two arrays to add them to the system
rightSerials=("serial1" "serial2")
leftSerials=("serial1" "serial2")

export AR_RIGHTCAM_CONNECTED=false
export AR_LEFTCAM_CONNECTED=false

## For each Flea3 camera attached
while read line
do
    ## Parse bus number and device number from lsusb results
    BUS=$(echo $line | grep -o -E 'Bus [0-9]*' | grep -o -E '[0-9]*')
    DEVICE=$(echo $line | grep -o -E 'Device [0-9]*' | grep -o -E '[0-9]*')
    
    ## Query device for hexadecimal serial number
    SERIAL=$(udevadm info --query=property /dev/bus/usb/$BUS/$DEVICE | grep "ID_SERIAL_SHORT" | sed 's/ID_SERIAL_SHORT=//g')
    
    ## Covert hexadecimal serial number to decimal format
    SERIAL=$((0x$SERIAL))

    ## Check serial number against list of right camera serials
    containsElement ${SERIAL} "${rightSerials[@]}"
    if [ $? == 0 ]; then
        export AR_RIGHTCAM=${SERIAL}
        export AR_RIGHTCAM_CONNECTED=true
    fi

    ## Check serial number against list of left camera serials
    containsElement ${SERIAL} "${leftSerials[@]}"
    if [ $? == 0 ]; then
        export AR_LEFTCAM=${SERIAL}
        export AR_LEFTCAM_CONNECTED=true
    fi

done <<< "$(lsusb | grep '1e10:\(3300\|300a\)')" # Use lsusb to find all PointGrey Flea3 cameras plugged in right now

if [[ $DEBUG == true ]]; then
    echo "AR_RIGHTCAM           = ${AR_RIGHTCAM}"
    echo "AR_RIGHTCAM_CONNECTED = ${AR_RIGHTCAM_CONNECTED}"
    echo "AR_LEFTCAM            = ${AR_LEFTCAM}"
    echo "AR_LEFTCAM_CONNECTED  = ${AR_LEFTCAM_CONNECTED}"
fi

