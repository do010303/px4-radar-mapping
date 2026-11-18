# Shared by the launchers of this example: other simulations on this machine
# (the ArduPilot rover work in particular) come first.
#
#   . sim_guard.sh
#   wait_for_free_sim     # returns once nothing else has run for QUIET_TIME s
#   other_sims            # pids of other simulations, empty when free
#
# Needs BUILD (PX4 build dir) and RUN_DIR (this example's run dir) set.

# Any other simulator: Gazebo started with `gz sim` (ours is titled
# px4_mapping_gazebo, see gz_launch.rb), an ArduPilot SITL or its launchers, or a PX4
# SITL other than ours. Prints their pids.
other_sims() {
	# Anchored at the start of the command line, so that a shell merely
	# mentioning these names (an editor, this very check) does not count.
	pgrep -f "^gz sim |^[^ ]*/ardu(rover|copter|plane|sub) |^[^ ]*python[0-9.]* [^ ]*sim_vehicle|^(timeout [0-9]+ |/bin/bash |bash )*[^ ]*sim_check/rig.sh" || true

	# Any PX4 SITL build (default, us_d1, ...), so that a build of another variant is not missed.
	for pid in $(pgrep -f "^$(dirname "$BUILD")/px4_sitl_[^/ ]*/bin/px4 "); do
		tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q -- "-w $RUN_DIR/rootfs" || echo "$pid"
	done
}

# Returns once the machine has been free of other simulations for QUIET_TIME
# seconds (automated runs come in series with short gaps). WAIT_TIMEOUT=<s>
# gives up (returns 1) after that long; default: wait indefinitely.
wait_for_free_sim() {
	QUIET_TIME="${QUIET_TIME:-60}"
	WAITED=0
	QUIET=0

	while [ "$QUIET" -lt "$QUIET_TIME" ]; do
		BUSY=$(other_sims)

		if [ -n "$BUSY" ]; then
			if [ "$QUIET" -gt 0 ] || [ "$WAITED" -eq 0 ]; then
				echo "$(date +%T) another simulation is running; waiting for it to finish:" >&2
				echo "$BUSY" | xargs -r ps -o pid=,args= -p | cut -c1-120 >&2
			fi
			QUIET=0
		else
			# Say so once, so that a silent wait does not look like a hang.
			[ "$QUIET" -eq 0 ] && echo "$(date +%T) no other simulation; starting once it stays so for ${QUIET_TIME} s" \
				"(QUIET_TIME=5 to wait less)" >&2
			QUIET=$((QUIET + 5))
		fi

		if [ -n "${WAIT_TIMEOUT:-}" ] && [ "$WAITED" -ge "$WAIT_TIMEOUT" ]; then
			echo "Still busy after ${WAITED} s; giving up." >&2
			return 1
		fi

		[ "$QUIET" -lt "$QUIET_TIME" ] && sleep 5 && WAITED=$((WAITED + 5))
	done

	if [ "$WAITED" -gt "$QUIET_TIME" ]; then
		echo "$(date +%T) simulator free; starting." >&2
	fi
}
