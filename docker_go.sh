
PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../

DOCKER_REPO=apm-dev

CCACHE_DIR=${HOME}/.ccache
mkdir -p "${CCACHE_DIR}"

docker run -it --rm -w "${SRC_DIR}" \
        --env=CCACHE_DIR="${CCACHE_DIR}" \
        --volume=${CCACHE_DIR}:${CCACHE_DIR}:rw \
        --volume=${SRC_DIR}:${SRC_DIR}:rw \
        ${DOCKER_REPO} /bin/bash -c "$1 $2 $3"
