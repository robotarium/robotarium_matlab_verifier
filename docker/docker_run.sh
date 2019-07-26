# 1: Robotarium host IP


if [ "$1" == "" ]; then
	echo "Argument 1 must be Robotarium verifier config path"
	exit
fi

sudo docker run -ti --rm --name verifier \
        --mac-address="98:90:96:cf:c7:c8" \
        --env="DISPLAY" \
	    --env "ROBOTARIUM_VERIFIER_CONFIG=$1" \
        -v /tmp/.X11-unix/:/tmp/.X11-unix/:rw \
	robotarium:matlab_verifier
