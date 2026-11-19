#!/bin/bash

# test script for ioc-template to verify that the container loads and the
# generic IOC will start - demonstrating that the correct runtime libraries
# all present and correct and that mounting IOC config or ibek config
# works as expected.

TAG=${1} # pass a tag on the command line to test a prebuilt image
THIS=$(realpath $(dirname $0))
ROOT=$(realpath ${THIS}/..)
CONF=/epics/ioc/config

# log commands and stop on errorsr
set -ex

# prefer docker but use podman if USE_PODMAN is set
if docker version &> /dev/null && [[ -z $USE_PODMAN ]]
    then docker=docker
    else docker=podman
fi

cd ${ROOT}

# if a tag was passed in this implies it was already built
export TAG=${TAG:-ec_test}
if [[ ${TAG} == "ec_test" ]] ; then TARGET=runtime ./build; fi

# try out a test ibek config IOC instance with the generic IOC
mounts=" -v ${THIS}/config:${CONF}:ro"
# allows local testing of a modified start.sh script
mounts+=" -v ${ROOT}/ioc/start.sh:/epics/ioc/start.sh:ro"
opts="--rm --security-opt=label=disable"
result=$($docker run ${opts} ${mounts} ${TAG} /epics/ioc/start.sh 2>&1)

# check that the IOC output expected results
if echo "${result}" | grep -i error; then
    echo "ERROR: errors in IOC startup"
    exit 1
elif [[ ! ${result} =~ "5.15" || ! ${result} =~ "/epics/runtime/st.cmd" ]]; then
    echo "ERROR: dbgf output not as expected"
    exit 1
fi

# the NI USB-621x on its fake: Ao2 -> Ai3 and Bo1 -> Bi1 through the driver
if ! echo "${result}" | grep -A1 "dbgf NITEST:Ai3" | grep -Eq "DBF_DOUBLE: *2\.500"; then
    echo "ERROR: the NI USB-621x fake's Ao2 -> Ai3 not as expected"
    exit 1
elif ! echo "${result}" | grep -A1 "dbgf NITEST:Model" | grep -q "USB-6211"; then
    echo "ERROR: the NI USB-621x fake's model not as expected"
    exit 1
elif ! echo "${result}" | grep -A1 "dbgf NITEST:Bi1" | grep -q "\"High\""; then
    echo "ERROR: the NI USB-621x fake's Bo1 -> Bi1 not as expected"
    exit 1
elif ! echo "${result}" | grep -A1 "dbgf NITEST:WaveDig3" | grep -Eq "DBF_DOUBLE\[100\]: *2\.500"; then
    echo "ERROR: the NI USB-621x fake's timed scan (WaveDig3) not as expected"
    exit 1
elif ! echo "${result}" | grep -A1 "dbgf NITEST2:Model" | grep -q "USB-6210" ||
     ! echo "${result}" | grep -A1 "dbgf NITEST2:Ai4" | grep -Eq "DBF_DOUBLE: *0\.300"; then
    echo "ERROR: the NI USB-6210 fake not as expected"
    exit 1
fi

# the NI USB-6008 on its fake: Ao1 -> Ai1 (RSE, 10 mV steps), Bo1 0 -> Bi1
# Low (an undriven line reads High), a WaveDig scan, the model
if ! echo "${result}" | grep -A1 "dbgf NITEST3:Ai1" | grep -Eq "DBF_DOUBLE: *2\.(49|50)"; then
    echo "ERROR: the NI USB-6008 fake's Ao1 -> Ai1 not as expected"
    exit 1
elif ! echo "${result}" | grep -A1 "dbgf NITEST3:Model" | grep -q "USB-6008"; then
    echo "ERROR: the NI USB-6008 fake's model not as expected"
    exit 1
elif ! echo "${result}" | grep -A1 "dbgf NITEST3:Bi1" | grep -q "\"Low\""; then
    echo "ERROR: the NI USB-6008 fake's Bo1 -> Bi1 not as expected"
    exit 1
elif ! echo "${result}" | grep -A1 "dbgf NITEST3:WaveDig1" | grep -Eq "DBF_DOUBLE\[100\]: *2\.(49|50)"; then
    echo "ERROR: the NI USB-6008 fake's timed scan (WaveDig1) not as expected"
    exit 1
fi

# verify --test mode generates runtime assets but does not launch the IOC binary.
# KUBERNETES_PORT is set as it is inside a pod (e.g. kodman on GitLab CI),
# where start.sh would otherwise re-exec through stdio-socket.
test_result=$($docker run ${opts} -e KUBERNETES_PORT=tcp://ci:443 ${mounts} ${TAG} /epics/ioc/start.sh --test 2>&1)

if echo "${test_result}" | grep -i error; then
    echo "ERROR: errors in IOC --test startup"
    exit 1
elif ! echo "${test_result}" | grep -q "Test mode:"; then
    echo "ERROR: --test did not report test mode"
    exit 1
fi

echo "Tests passed!"
