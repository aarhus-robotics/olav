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

# Exit on non-zero statuses.
set -e

# Capture interrupt signals.
trap interrupt INT
function interrupt() {
    exit 1
}

autorossource() {
    source /opt/ros/humble/setup.zsh &&
        source ${HOME}/ROS/install/setup.zsh &&
        export ROS_LOG_DIR=${HOME}/ROS/log/run &&
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
        export CYCLONEDDS_URI=file://${HOME}/ROS/config/cyclonedds/cyclonedds.xml

}

check-ros-version() {
    if [ -z "${ROS_VERSION}" ]; then
        prettyprint "Sourcing ROS environment ..."
        autorossource
    fi
}

get-dependencies() {
    if [[ -z "$(command -v lolcat)" ]]; then
        printf "Installing dependency \"lolcat\" ...\n"
        sudo apt install --yes lolcat
    fi
}

has-session() {
    if tmux has-session -t ${1} >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

# Pretty print strings using lolcat
prettyprint() {
    printf "${1}\n" | lolcat
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
    timeout 10s git -C ${HOME}/ROS/src/${1} fetch
    git -C ${HOME}/ROS/src/${1} reset --hard origin/$(git -C ${HOME}/ROS/src/${1} symbolic-ref --short HEAD)
    return 0
}

ros-git-checkout() {
    timeout 10s git -C ${HOME}/ROS/src/${1} fetch origin
    git -C ${HOME}/ROS/src/${1} reset --hard origin/${2}
    return 0
}


LIST_OF_REPOSITORIES=("aarhus-robotics/olav" "aarhus-robotics/navi")
LIST_OF_ODIN_SESSIONS=("datalogger" "description" "drive-by-wire" "navigation" "perception" "drawbar")
LIST_OF_THOR_SESSIONS=("peripherals")

#####################################
# Install the required dependencies #
#####################################
get-dependencies

##############################################
# Check the environment is correctly sourced #
##############################################
check-ros-version

################################
# Validate the device hostname #
################################
if
    [ "$(cat /etc/hostname)" = "odin" ]
then
    VALID_SESSIONS=(${LIST_OF_ODIN_SESSIONS})
elif [ "$(cat /etc/hostname)" = "thor" ]; then
    VALID_SESSIONS=(${LIST_OF_THOR_SESSIONS})
else
    prettyprint "Invalid hostname.\n"
    exit 1
fi

if [ "${1}" = "sessions" ]; then

    ########################
    # Start a tmux session #
    ########################
    if [ "${2}" = "start" ]; then
        if is_in_list "${VALID_SESSIONS}" ${3}; then
            prettyprint "Starting session \"${3}\" session ..."
            if [ "${3}" = "description" ]; then
                ros2 launch olav_description description.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "drive-by-wire" ]; then
                ros2 launch olav_control drive_by_wire.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "perception" ]; then
                ros2 launch olav_sensors perception.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "navigation" ]; then
                ros2 launch olav_sensors navigation.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "mapping" ]; then
                ros2 launch olav_sensors mapping.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "drawbar" ]; then
                ros2 launch olav_sensors drawbar.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "datalogger" ]; then
                ros2 launch olav_utilities datalogger.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "peripherals" ]; then
                ros2 launch olav_peripherals peripherals.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                exit 0
            elif [ "${3}" = "autonomy" ]; then
                if [ "${4}" = "navi" ]; then
                    ros2 launch olav_launch navi.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                    exit 0
                elif [ "${4}" = "nato" ]; then
                    ros2 launch olav_launch nato.launch.py parameters_overrides:=${HOME}/ROS/config/parameters/olav_parameters_overrides.yaml
                    exit 0
                fi
            else
                prettyprint "Invalid session name \"${3}\"!"
                exit 1
            fi
        else
            prettyprint "Invalid session name \"${3}\"!"
            exit 1
        fi

    #######################
    # Stop a tmux session #
    #######################
    elif [ "${2}" = "stop" ]; then
        if is_in_list "${VALID_SESSIONS}" ${3}; then
            if has-session ${3}; then
                prettyprint "Stopping session \"${3}\"..."
                tmux send-keys -t ${3} C-c
                tmux send -t ${3} exit Enter
                exit 0
            else
                prettyprint "Session \"${3}\" is not running!"
                exit 0
            fi
        else
            prettyprint "Invalid session name \"${3}\"!"
            exit 1
        fi

    ############################
    # Attach to a tmux session #
    ############################
    elif [ "${2}" = "attach" ]; then
        if is_in_list "${VALID_SESSIONS}" ${3}; then
            if has-session ${3}; then
                prettyprint "Attaching to session \"${3}\" ..."
                tmux attach -t ${3}
                exit 0
            else
                prettyprint "Session \"${3}\" is not running!"
                exit 0
            fi
        else
            prettyprint "Invalid session name \"${3}\"!"
            exit 1
        fi

    ####################
    # Mux tmux session #
    ####################
    elif [ "${2}" = "mux" ]; then
        if is_in_list "${VALID_SESSIONS}" ${3}; then
            if ! has-session ${3}; then
                prettyprint "Muxing session \"${3}\" ..."
                tmux new-session -d -e ROS_VERSION= -s ${3} \; send "olav sessions start ${3}" Enter
                exit 0
            else
                prettyprint "Session \"${3}\" is already running!"
                exit 0
            fi
        else
            prettyprint "Invalid session name \"${3}\"!"
            exit 1
        fi

    ######################
    # List tmux sessions #
    ######################
    elif [ "${2}" = "list" ]; then
        prettyprint "Listing open sessions ..."
        tmux list-sessions | lolcat
        exit 0

    elif [ "${2}" = "kill" ]; then
        prettyprint "Killing all sessions ..."
        tmux kill-server
        exit 0
    fi

#######################
# Checkout Git branch #
#######################
elif [ "${1}" = "checkout" ]; then
    prettyprint "Checking out branch \"${2}\" ..."
    ros-git-checkout aarhus-robotics/olav ${2} | lolcat
    exit 0


###################
# Update packages #
###################
elif [ "${1}" = "update" ]; then
    for repository in ${LIST_OF_REPOSITORIES}; do
        prettyprint "Updating repository \"$(echo ${repository} | cut -d '/' -f2)\" ..."
        ros-git-fetch ${repository} | lolcat
    done
    exit 0

##################
# Build packages #
##################
elif [ "${1}" = "build" ]; then
    prettyprint "Building ROS packages ..."
    cd ${HOME}/ROS
    colcon --log-base ${HOME}/ROS/log/build build | lolcat >/dev/null
    exit 0

# GUI
elif [ "${1}" = "gui" ]; then
    ###################
    # Foxglove Studio #
    ###################
    if [ "${2}" = "foxglove-studio" ]; then
        prettyprint "Launching Foxglove Studio ..."
        export ROS_PACKAGE_PATH=${HOME}/ROS/install
        foxglove-studio
        exit 0

    #################################
    # navi cheatsheet menu terminal #
    #################################
    elif [ "${2}" = "navi" ]; then
        prettyprint "Launching navi menu ..."
        konsole -e olav menu
        exit 0

    ###############
    # PlotJuggler #
    ###############
    elif [ "${2}" = "plotjuggler" ]; then
        prettyprint "Launching PlotJuggler ..."
        ros2 run plotjuggler plotjuggler -- -n
        exit 0

    ###########
    # RQt GUI #
    ###########
    elif [ "${2}" = "rqt_gui" ]; then
        prettyprint "Launching RQt GUI ..."
        ros2 run rqt_gui rqt_gui -- \
            --clear-config \
            --freeze-layout \
            --hide-title \
            --lock-perspective \
            --perspective-file "${HOME}/ROS/src/aarhus-robotics/olav/config/rqt_gui/olav.perspective" \
            --qt-binding pyqt
        exit 0

    ##################
    # RQt Image View #
    ##################
    elif [ "${2}" = "rqt_image_view" ]; then
        prettyprint "Launching RQt Image View ..."
        ros2 run rqt_image_view rqt_image_view -- \
            --qt-binding pyqt \
            --clear-config \
            --hide-title
        exit 0

    ########
    # RViz #
    ########
    elif [ "${2}" = "rviz" ]; then
        prettyprint "Launching RViz ..."
        # Fix graphical issues with Qt/OGRE applications.
        export QT_ENABLE_HIGHDPI_SCALING=0
        export QT_SCREEN_SCALE_FACTORS=1
        ros2 run rviz2 rviz2 -- -d ${HOME}/ROS/src/aarhus-robotics/olav/config/rviz/olav.rviz
        exit 0
    fi

#############
# navi menu #
#############
elif [ "${1}" = "menu" ]; then
    prettyprint "Loading navi menu ..."
    navi
    exit 0

elif [ "${1}" = "parameter" ]; then

    if [ ${2} = "get" ]; then
        prettyprint "Getting parameter \"${3}\" value...\n"
        prettyprint "%s.%s = %s" $(ros2 param get ${3} ${4} --hide-type)
        exit 0
    fi

elif [ "${1}" = "publish" ]; then

    if [ "${2}" = "ackermann" ]; then
        prettyprint "Publishing AckermannDriveStamped message ..."
        prettyprint "[ ACKERMANNDRIVESTAMPED INFO | Speed: %0.2f | Steering %0.2f ]" ${speed} ${steering}
        ros2 topic pub /olav/multiplexer/in/drive \
            ackermann_msgs/msg/AckermannDriveStamped \
            "{"header": {"frame_id": "autonomy", "stamp": "now"}, "drive": {"speed": "${3}", "steering_angle": $((${4} * 3.141592 / 180))}}" \
            -r ${5} \
            >/dev/null 2>&1
        exit 0

    elif [ "${2}" = "heartbeat" ]; then
        prettyprint "Sending heartbeat ..."
        prettyprint "[ HEARTBEAT INFO | Authority: \"%s\" | Rate: %d ]" ${1} ${2}
        ros2 topic pub /olav/multiplexer/in/heartbeat \
            std_msgs/msg/Header \
            "{"frame_id": "${3}", "stamp": "now"}" \
            -r ${4} \
            >/dev/null 2>&1
        exit 0

    elif [ "${2}" = "setpoint" ]; then
        prettyprint "Sending setpoint ..."
        prettyprint "[ SETPOINT INFO | Magnitude: \"%d\" ]" ${4}
        ros2 topic pub /olav/controls/${3} \
            olav_interfaces/msg/SetpointStamped \
            "{"header": {"frame_id": "console", "stamp": "now"}, "setpoint": ${4}}"
        exit 0
    fi

#######################
# Datalogger controls #
#######################
elif [ "${1}" = "record" ]; then

    ######################################
    # > Start a new datalogger recording #
    ######################################
    if [ "${2}" = "start" ]; then
        prettyprint "Starting recording..."
        ros2 service call /olav/datalogger/start \
            std_srvs/srv/Trigger \
            "{}" \
            >/dev/null 2>&1 &
        exit 0

    ###########################################
    # > Stop the current datalogger recording #
    ###########################################
    elif [ "${2}" = "stop" ]; then
        prettyprint "Stopping recording ...\n"
        ros2 service call /olav/datalogger/stop \
            std_srvs/srv/Trigger \
            "{}" \
            >/dev/null 2>&1 &
        exit 0

    ############################################################
    # > Upload a datalogger recording to the external storage #
    ###########################################################
    elif [ "${2}" = "upload" ]; then

        ###############################################################
        # >> Upload all datalogger recordings to the external storage #
        ###############################################################
        if [ "${3}" = "all" ]; then
            sudo mount /dev/sda1 ${HOME}/Portable
            prettyprint "Uploading all recordings ..."
            rsync -aPu ${HOME}/ROS/bags/** ${HOME}/Portable/
            sudo umount ${HOME}/Portable
            exit 0

        #####################################################################
        # >> Upload a specific datalogger recording to the external storage #
        #####################################################################
        else
            sudo mount /dev/sda1 ${HOME}/Portable
            prettyprint "Uploading recording ${3} ..."
            rsync -aPu ${HOME}/ROS/bags/${3} ${HOME}/ERDA/Inbox/
            sudo umount ${HOME}/Portable
            exit 0
        fi

    ###################################
    # > Delete a datalogger recording #
    ###################################
    elif [ "${2}" = "delete" ]; then

        if [ "${3}" = "all" ]; then
            prettyprint "Deleting all recordings ..."
            rm -rf ${HOME}/ROS/bags/** >/dev/null 2>&1
            exit 0

        else
            prettyprint "Deleting recording ${3}..."
            rm -rf ${HOME}/ROS/bags/${3} >/dev/null 2>&1
            exit 0
        fi

    ##############################################
    # > Edit the datalogger recorded topics list #
    ##############################################
    elif [ "${2}" = "topics" ]; then

        ##############################################
        # >> Get the datalogger recorded topics list #
        ##############################################
        if [ "${3}" = "get" ]; then
            prettyprint "Retrieving datalogger topics..."
            ros2 param get /olav/datalogger topics --hide-type | sed 's:^.\(.*\).$:\1:' | tr "," "\n" | tr -d " "
            exit 0

        ##############################################
        # >> Set the datalogger recorded topics list #
        ##############################################
        elif [ "${3}" = "set" ]; then
            prettyprint "Setting datalogger topics..."
            ros2 param set /olav/datalogger_node topics "[$(/home/olav/.local/bin/yq ".topics" ROS/src/aarhus-robotics/olav/olav_utilities/olav_utilities/data/datalogger/presets/${4}.yaml --output-format tsv | sed -e 's/\t/,/g')]"
            exit 0
        fi

    ##############################################
    # > Check available datalogger storage space #
    ##############################################
    elif [ "${2}" = "storage" ]; then
        prettyprint "Available space: %s" \
            $(df -H ${HOME}/ROS/bags | tail -1 | awk '{print $4}')
        exit 0

    else
        prettyprint "No valid verb received - accepted verbs are <start>, <stop>, <upload>, <delete>, <topics> or <storage>."
        exit 1
    fi

elif [ "${1}" = "sensors" ]; then

    if [ "${2}" = "ins" ]; then

        if [ "${3}" = "reset" ]; then
            prettyprint "Resetting EKF ..."
            ros2 service call /olav/sensors/inertial_navigation_system/filter/reset std_srvs/srv/Empty "{}"
            exit 0

        else
            prettyprint "Invalid inertial navigation system command."
            exit 1
        fi

    ########################
    # LiDAR sensor control #
    ########################
    elif [ "${2}" = "lidar" ]; then

        #################################
        # > Get the LiDAR sensor status #
        #################################
        if [ "${3}" = "status" ]; then
            prettyprint "Getting LiDAR status ..."
            echo "get_sensor_info" | netcat -N saga.lan 7501 | python3 -m json.tool | lolcat
            exit 0

        ###################################################
        # > Set the LiDAR sensor operating mode to NORMAL #
        ###################################################
        elif [ "${3}" = "start" ]; then
            prettyprint "Starting LiDAR sensor ..."
            echo "set_config_param operating_mode NORMAL" | netcat -N saga.lan 7501 >/dev/null 2>&1
            echo "reinitialize" | netcat -N saga.lan 7501 >/dev/null 2>&1
            exit 0

        ####################################################
        # > Set the LiDAR sensor operating mode to STANDBY #
        ####################################################
        elif [ "${3}" = "stop" ]; then
            prettyprint "Stopping LiDAR sensor ..."
            echo "set_config_param operating_mode STANDBY" | netcat -N saga.lan 7501 >/dev/null 2>&1
            echo "reinitialize" | netcat -N saga.lan 7501 >/dev/null 2>&1
            exit 0

        else
            prettyprint "Invalid verb - must be one of the following: (<start>, <stop>, <status>)."
            exit 1
        fi
    else
        prettyprint "Invalid sensor name: ${2}."
        exit 1
    fi

########################
# Time synchronization #
########################

elif [ "${1}" = "time" ]; then

    ################
    # Time sources #
    ################
    if [ "${2}" = "sources" ]; then
        prettyprint "Displaying time sources...\n"
        watch -n 1 "chronyc sources -v"
        exit 0

    #######################
    # Tracking statistics #
    #######################
    elif [ "${2}" = "tracking" ]; then
        prettyprint "Displaying tracking statistics...\n"
        watch -n 1 "chronyc tracking -v"
        exit 0
    fi

###################
# System commands #
###################
elif [ "${1}" = "system" ]; then

    if [ "$(cat /etc/hostname)" != "thor" ]; then
        prettyprint "System commands may only be used from thor.olavnet!"
        exit 1
    fi

    ###########################
    # Switch system to branch #
    ###########################

    if [ "${2}" = "checkout" ]; then
        prettyprint "Checking out on thor.lan ..."
        olav checkout ${3}

        prettyprint "Checking out on odin.lan ..."
        ssh -qt odin.lan "olav checkout ${3}"
        exit 0
    fi

    #########################
    # Build at system level #
    #########################

    if [ "${2}" = "build" ]; then
        prettyprint "Building on thor.lan ..."
        olav build ${3}

        prettyprint "Building on odin.lan ..."
        ssh -qt odin.lan "olav build ${3}"
        exit 0
    fi

    ##########################
    # Update at system level #
    ##########################

    if [ "${2}" = "update" ]; then
        prettyprint "Updating on thor.lan ..."
        olav update ${3}
    
        prettyprint "Updating on odin.lan ..."
        ssh -qt odin.lan "olav update ${3}"
        exit 0
    fi

    #######################
    # Bring the system up #
    #######################
    if [ "${2}" = "up" ]; then

        # :: Start the local session
        prettyprint "Bringing system up on thor.lan ..."
        olav sessions mux peripherals

        # :: Start the main computing unit sessions.
        prettyprint "Bringing system up on odin.lan ..."
        ssh -qt odin.lan "olav sessions mux datalogger"
        ssh -qt odin.lan "olav sessions mux description"
        ssh -qt odin.lan "olav sessions mux drive-by-wire"
        ssh -qt odin.lan "olav sessions mux navigation"
        ssh -qt odin.lan "olav sessions mux perception"

        exit 0

    #########################
    # Bring the system down #
    #########################
    elif [ "${2}" = "down" ]; then

        # :: Stop the main computing unit sessions.
        prettyprint "Bringing system down on odin.lan ..."
        ssh -qt odin.lan "olav sessions stop datalogger"
        ssh -qt odin.lan "olav sessions stop perception"
        ssh -qt odin.lan "olav sessions stop navigation"
        ssh -qt odin.lan "olav sessions stop drive-by-wire"
        ssh -qt odin.lan "olav sessions stop description"

        # :: Stop the local session
        prettyprint "Bringing system down on thor.lan ..."
        olav sessions stop peripherals

        # :: Set the LiDAR sensor to STANDBY mode.
        olav sensors lidar stop

        exit 0
    fi

else
    prettyprint "Invalid command \"${1}\"!"
fi
