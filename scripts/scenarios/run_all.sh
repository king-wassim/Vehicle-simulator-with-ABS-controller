#!/usr/bin/env bash
# Run every scenario sequentially. For each, start the plant-side script
# in the background, wait for the listening port, then launch the C ECU
# binary, wait for both to finish, collect exit codes.
#
# Returns 0 iff every scenario passed.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SCENARIOS=(s1_nominal s2_ice_patch s3_stuck_sensor s4_comm_loss s5_crc_corruption s6_noisy_sensor)
PORT=9000
overall=0

# A generous duration so the C ECU outlives the plant for every scenario.
ECU_DURATION_S=20

for s in "${SCENARIOS[@]}"; do
    echo
    echo "==================== $s ===================="
    python3 -m scripts.scenarios.$s &
    SERVER=$!

    # Wait up to 5 s for the plant to bind the port.
    for i in 1 2 3 4 5 6 7 8 9 10; do
        ss -tln 2>/dev/null | grep -q ":$PORT " && break
        sleep 0.5
    done

    ./controller/build/abs_ecu 127.0.0.1 $PORT $ECU_DURATION_S 100 \
        > /tmp/ecu_${s}.stdout 2> /tmp/ecu_${s}.stderr
    ECU_RC=$?
    wait $SERVER
    SRV_RC=$?

    echo "-- exit codes : plant=$SRV_RC  ECU=$ECU_RC"
    if [[ $SRV_RC -ne 0 ]]; then
        overall=1
    fi
done

echo
echo "============================================="
if [[ $overall -eq 0 ]]; then
    echo "ALL $((${#SCENARIOS[@]})) SCENARIOS PASSED"
else
    echo "SCENARIO RUN FAILED"
fi
exit $overall
