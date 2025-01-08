#!/usr/bin/zsh

#############################################################################
#                            _     _     _     _                            #
#                           / \   / \   / \   / \                           #
#                          ( O ) ( L ) ( A ) ( V )                          #
#                           \_/   \_/   \_/   \_/                           #
#                                                                           #
#                  OLAV: Off-Road Light Autonomous Vehicle                  #
#############################################################################
#
# This script manages the commands for computing units and human-interface
# devices in OLAV. The script must executed from a ROS sourced environment.

# Capture interrupt signals.
trap interrupt INT
function interrupt() {
    exit
}

# Check if a string is present in a list of strings.
is_in_list() {
    for element in ${VALID_SESSIONS}; do
        if [ "${element}" = "${2}" ]; then
            return 0
        fi
    done
    return 1
}

# Fetch the latest commit from origin/main and override any local changes for a
# repository under the ROS local workspace source directory.
ros-git-fetch() {
    timeout 10s git -C /home/olav/ROS/src/${1} fetch
    git -C /home/olav/ROS/src/${1} reset --hard origin/main
}

LIST_OF_REPOSITORIES=("aarhus-robotics/roto" "aarhus-robotics/olav" "aarhus-robotics/navi")
LIST_OF_ODIN_SESSIONS=("datalogger" "description" "drive-by-wire" "navigation" "perception" "drawbar")
LIST_OF_THOR_SESSIONS=("peripherals")

################################
# Validate the device hostname #
################################
if [ "$(cat /etc/hostname)" = "odin" ]; then
    VALID_SESSIONS=(${LIST_OF_ODIN_SESSIONS})
elif [ "$(cat /etc/hostname)" = "thor" ]; then
    VALID_SESSIONS=(${LIST_OF_THOR_SESSIONS})
else
    printf "Invalid hostname.\n"
    exit
fi

########################
# Start a tmux session #
########################
if [ "${1}" = "start" ]; then

    if is_in_list "${VALID_SESSIONS}" ${2}; then
        printf "Starting session \"${2}\" session ...\n"
        if [ "${2}" = "description" ]; then
            ros2 launch olav_description description.launch.py
        elif [ "${2}" = "drive-by-wire" ]; then
            ros2 launch olav_control drive_by_wire.launch.py
        elif [ "${2}" = "perception" ]; then
            ros2 launch olav_sensors perception.launch.py
        elif [ "${2}" = "navigation" ]; then
            ros2 launch olav_sensors navigation.launch.py
        elif [ "${2}" = "navigation" ]; then
            ros2 launch olav_sensors mapping.launch.py
        elif [ "${2}" = "drawbar" ]; then
            ros2 launch olav_sensors drawbar.launch.py
        elif [ "${2}" = "datalogger" ]; then
            ros2 launch olav_launch datalogger.launch.py
        elif [ "${2}" = "peripherals" ]; then
            ros2 launch olav_peripherals peripherals.launch.py
        else
            printf "Invalid session name.\n"
        fi
    fi

#######################
# Stop a tmux session #
#######################
elif [ "${1}" = "stop" ]; then
    if is_in_list "${VALID_SESSIONS}" ${2}; then
        printf "Stopping session \"${2}\"...\n"
        tmux send-keys -t ${2} C-c >/dev/null 2>&1
    else
        printf "Invalid session name \"${2}\"!\n"
    fi

############################
# Attach to a tmux session #
############################
elif [ "${1}" = "attach" ]; then

    # Core session
    if is_in_list "${VALID_SESSIONS}" ${2}; then
        printf "Attaching to session \"${2}\" ...\n"
        tmux attach -t ${2}
    else
        printf "Invalid session name \"${2}\"!\n"
    fi

####################
# Mux tmux session #
####################
elif [ "${1}" = "mux" ]; then
    if is_in_list "${VALID_SESSIONS}" ${2}; then
        printf "Muxing session \"${2}\" ...\n"
        tmux new-session -d -s ${2} "olav start ${2}"
    else
        printf "Invalid session name \"${2}\"!\n"
    fi

###################
# Update packages #
###################
elif [ "${1}" = "update" ]; then
    for repository in ${LIST_OF_REPOSITORIES}; do
        printf "Updating repository \"$(echo ${repository} | cut -d '/' -f2)\" ...\n"
        ros-git-fetch ${repository}
    done

##################
# Build packages #
##################
elif [ "${1}" = "build" ]; then
    printf "Building ROS packages ...\n"
    cd /home/olav/ROS
    colcon --log-base /home/olav/ROS/log/build build >/dev/null

# GUI
elif [ "${1}" = "gui" ]; then
    ###################
    # Foxglove Studio #
    ###################
    if [ "${2}" = "foxglove-studio" ]; then
        export ROS_PACKAGE_PATH=/home/olav/ROS/install
        foxglove-studio

    #################################
    # navi cheatsheet menu terminal #
    #################################
    elif [ "${2}" = "navi" ]; then
        konsole -e olav menu

    ###############
    # PlotJuggler #
    ###############
    elif [ "${2}" = "plotjuggler" ]; then
        ros2 run plotjuggler plotjuggler -- -n

    ###########
    # RQt GUI #
    ###########
    elif [ "${2}" = "rqt_gui" ]; then
        ros2 run rqt_gui rqt_gui -- \
            --clear-config \
            --freeze-layout \
            --hide-title \
            --lock-perspective \
            --perspective-file "/home/olav/ROS/src/aarhus-robotics/olav/config/rqt_gui/olav.perspective" \
            --qt-binding pyqt

    ##################
    # RQt Image View #
    ##################
    elif [ "${2}" = "rqt_image_view" ]; then
        ros2 run rqt_image_view rqt_image_view -- \
            --qt-binding pyqt \
            --clear-config \
            --hide-title

    ########
    # RViz #
    ########
    elif [ "${2}" = "rviz" ]; then
        # Fix graphical issues with Qt/OGRE applications.
        export QT_ENABLE_HIGHDPI_SCALING=0
        export QT_SCREEN_SCALE_FACTORS=1
        ros2 run rviz2 rviz2 -- -d /home/olav/ROS/src/aarhus-robotics/olav/config/rviz/olav.rviz
    fi

#############
# navi menu #
#############
elif [ "${1}" = "menu" ]; then
    printf "Loading menu...\n"
    while true; do
        navi --path /home/olav/.config/navi/cheats
        printf "\nPress any key to return to the menu."
        read
    done

elif [ "${1}" = "parameter" ]; then

    if [ ${2} = "get" ]; then
        printf "Getting parameter \"${3}\" value...\n"
        printf "%s.%s = %s" $(ros2 param get ${3} ${4} --hide-type)
    fi

elif [ "${1}" = "publish" ]; then

    if [ "${2}" = "ackermann" ]; then
        printf "Publishing AckermannDriveStamped message ...\n"
        printf "[ ACKERMANNDRIVESTAMPED INFO | Speed: %0.2f | Steering %0.2f ]\n" ${speed} ${steering}
        ros2 topic pub /olav/commands/composite/drive \
            ackermann_msgs/msg/AckermannDriveStamped \
            "{"header": {"frame_id": "autonomy", "stamp": "now"}, "drive": {"speed": "${3}", "steering_angle": $((${4} * 3.141592 / 180))}}" \
            -r ${5} \
            >/dev/null 2>&1

    elif [ "${2}" = "heartbeat" ]; then
        printf "Sending heartbeat ...\n"
        printf "[ HEARTBEAT INFO | Authority: \"%s\" | Rate: %d ]\n" ${1} ${2}
        ros2 topic pub /olav/commands/signals/heartbeat \
            std_msgs/msg/Header \
            "{"frame_id": "${3}", "stamp": "now"}" \
            -r ${4} \
            >/dev/null 2>&1

    elif [ "${2}" = "setpoint" ]; then
        printf "Sending setpoint ...\n"
        printf "[ SETPOINT INFO | Magnitude: \"%d\" ]\n" ${4}
        ros2 topic pub /olav/controls/${3} \
            olav_interfaces/msg/SetpointStamped \
            "{"header": {"frame_id": "console", "stamp": "now"}, "setpoint": ${4}}"
    fi

#######################
# Datalogger controls #
#######################
elif [ "${1}" = "record" ]; then

    ######################################
    # > Start a new datalogger recording #
    ######################################
    if [ "${2}" = "start" ]; then
        printf "Starting recording..."
        ros2 service call /olav/datalogger/start \
            std_srvs/srv/Trigger \
            "{}" \
            >/dev/null 2>&1 &

    ###########################################
    # > Stop the current datalogger recording #
    ###########################################
    elif [ "${2}" = "stop" ]; then
        printf "Stopping recording ...\n"
        ros2 service call /olav/datalogger/stop \
            std_srvs/srv/Trigger \
            "{}" \
            >/dev/null 2>&1 &

    ###########################################
    # > Upload a datalogger recording to ERDA #
    ###########################################
    elif [ "${2}" = "upload" ]; then
        mount /home/olav/ERDA

        ###############################################
        # >> Upload all datalogger recordings to ERDA #
        ###############################################
        if [ "${3}" = "all" ]; then
            printf "Uploading all recordings ...\n"
            rsync -aPu /home/olav/ROS/bags/** /home/olav/ERDA/Inbox/

        #####################################################
        # >> Upload a specific datalogger recording to ERDA #
        #####################################################
        else
            printf "Uploading recording ${3}..."
            rsync -aPu /home/olav/ROS/bags/${3} /home/olav/ERDA/Inbox/
        fi

    ###################################
    # > Delete a datalogger recording #
    ###################################
    elif [ "${2}" = "delete" ]; then
        if [ "${3}" = "all" ]; then
            printf "Deleting all recordings..."
            rm -rf /home/olav/ROS/bags/** >/dev/null 2>&1
        else
            printf "Deleting recording ${3}..."
            rm -rf /home/olav/ROS/bags/${3} >/dev/null 2>&1
        fi

    ##############################################
    # > Edit the datalogger recorded topics list #
    ##############################################
    elif [ "${2}" = "topics" ]; then

        ##############################################
        # >> Get the datalogger recorded topics list #
        ##############################################
        if [ "${3}" = "get" ]; then
            printf "Retrieving datalogger topics..."
            ros2 param get /olav/datalogger topics --hide-type | sed 's:^.\(.*\).$:\1:' | tr "," "\n" | tr -d " "

        ##############################################
        # >> Set the datalogger recorded topics list #
        ##############################################
        elif [ "${3}" = "set" ]; then
            printf "Setting datalogger topics..."
            ros2 param set /olav/datalogger topics "$(cat /home/olav/ROS/config/datalogger/${4})"
        fi

    ##############################################
    # > Check available datalogger storage space #
    ##############################################
    elif [ "${2}" = "storage" ]; then
        printf "Available space: %s\n" \
            $(df -H /home/olav/ROS/bags | tail -1 | awk '{print $4}')

    else
        echo "No valid verb received - accepted verbs are <start>, <stop>, <upload>, <delete>, <topics> or <storage>."
    fi

elif [ "${1}" = "sensor" ]; then
    if [ "${2}" = "ins" ]; then

        if [ "${3}" = "reset" ]; then
            printf "Resetting EKF ...\n"
            ros2 service call /olav/mip/ekf/reset std_srvs/srv/Empty "{}"
        else
            printf "Invalid INS command."
        fi

    ########################
    # LiDAR sensor control #
    ########################
    elif [ "${2}" = "lidar" ]; then
        #################################
        # > Get the LiDAR sensor status #
        #################################
        if [ "${3}" = "status" ]; then
            printf "Getting LiDAR status ...\n"
            echo "get_sensor_info" | netcat -N saga.lan 7501 | python3 -m json.tool

        ###################################################
        # > Set the LiDAR sensor operating mode to NORMAL #
        ###################################################
        elif [ "${3}" = "start" ]; then
            printf "Starting LiDAR sensor...\n"
            echo "set_config_param operating_mode NORMAL" | netcat -N saga.lan 7501 >/dev/null 2>&1
            echo "reinitialize" | netcat -N saga.lan 7501 >/dev/null 2>&1

        ####################################################
        # > Set the LiDAR sensor operating mode to STANDBY #
        ####################################################
        elif [ "${3}" = "stop" ]; then
            printf "Stopping LiDAR sensor...\n"
            echo "set_config_param operating_mode STANDBY" | netcat -N saga.lan 7501 >/dev/null 2>&1
            echo "reinitialize" | netcat -N saga.lan 7501 >/dev/null 2>&1

        else
            echo "Invalid verb - must be one of the following (<start>, <stop>, <status>).\n"
        fi
    else
        printf "Invalid sensor name."
    fi

########################
# Time synchronization #
########################

elif [ "${1}" = "time" ]; then
    ################
    # Time sources #
    ################
    if [ "${2}" = "sources" ]; then
        printf "Displaying time sources...\n"
        watch -n 1 "chronyc sources -v"

    #######################
    # Tracking statistics #
    #######################
    elif [ "${2}" = "tracking" ]; then
        printf "Displaying tracking statistics...\n"
        watch -n 1 "chronyc tracking -v"
    fi

###################
# System commands #
###################
elif [ "${1}" = "system" ]; then

    #######################
    # Bring the system up #
    #######################
    if [ "${2}" = "up" ]; then

        # Update the local ROS distribution
        olav update
        olav build

        # :: Start the local session
        olav mux peripherals

        # :: Update the main computing unit ROS distribution
        ssh -qt odin.lan "olav update"
        ssh -qt odin.lan "olav build"

        # :: Start the main computing unit sessions.
        ssh -qt odin.lan "olav mux datalogger"
        ssh -qt odin.lan "olav mux description"
        ssh -qt odin.lan "olav mux drive-by-wire"
        ssh -qt odin.lan "olav mux navigation"
        ssh -qt odin.lan "olav mux perception"

    #########################
    # Bring the system down #
    #########################
    elif [ "${2}" = "down" ]; then

        # :: Stop the local session
        olav stop peripherals

        # :: Stop the main computing unit sessions.
        ssh -qt odin.lan "olav stop datalogger"
        ssh -qt odin.lan "olav stop description"
        ssh -qt odin.lan "olav stop drive-by-wire"
        ssh -qt odin.lan "olav stop navigation"
        ssh -qt odin.lan "olav stop perception"

        # :: Set the LiDAR sensor to STANDBY mode.
        olav sensor lidar stop
    fi

else
    printf "Invalid command \"${1}\"\n"
fi
