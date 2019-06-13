ARG MATLAB_CONTAINER
FROM ${MATLAB_CONTAINER}

RUN apt-get update
RUN apt-get install -y git

ADD https://api.github.com/repos/robotarium/robotarium_matlab_verifier/git/refs/heads/master version.json
RUN git clone https://github.com/robotarium/robotarium_matlab_verifier.git && \
	cd robotarium_matlab_verifier && git submodule init && git submodule update

WORKDIR /robotarium_matlab_verifier
