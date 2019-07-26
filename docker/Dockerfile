ARG MATLAB_CONTAINER
FROM ${MATLAB_CONTAINER}

RUN apt-get update
RUN apt-get install -y git

ADD https://api.github.com/repos/robotarium/robotarium_matlab_verifier/git/refs/heads/updates version.json
RUN git clone https://github.com/robotarium/robotarium_matlab_verifier.git && \
	cd robotarium_matlab_verifier && git checkout master && git submodule update --init --recursive && git pull --recurse-submodules

WORKDIR /robotarium_matlab_verifier
