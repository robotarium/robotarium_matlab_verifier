ARG MATLAB_CONTAINER
FROM ${MATLAB_CONTAINER}

ADD https://api.github.com/repos/robotarium/robotarium_matlab_verifier/git/refs/heads/master version.json
RUN git clone https://github.com/robotarium/robotarium_matlab_verifier.git && \
	cd robotarium_matlab_verifier && git submodule init && git submodule update

WORKDIR /home/robotarium_matlab_verifier
