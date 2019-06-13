

if [ "$1" == "" ]; then
	echo "Argument 1 must be name of MATLAB container"
	exit
fi

sudo docker build --build-arg MATLAB_CONTAINER="$1" --tag robotarium:matlab_verifier .
