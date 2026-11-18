# Sourced by the launchers: render Gazebo (server and GUI) and RViz on the
# NVIDIA GPU of a hybrid-graphics laptop. Without this they render on the
# integrated GPU, where the Gazebo GUI is the bottleneck.
#
#   GPU=igpu ./run_sitl.sh       keep the integrated GPU
#
# Only PRIME render offload variables are set, for the processes started from
# here; the desktop and other simulations keep their GPU.

use_nvidia_gpu() {
	[ "${GPU:-}" = igpu ] && return 0
	[ -n "${__NV_PRIME_RENDER_OFFLOAD:-}" ] && return 0
	command -v nvidia-smi > /dev/null && nvidia-smi -L > /dev/null 2>&1 || return 0

	export __NV_PRIME_RENDER_OFFLOAD=1
	export __GLX_VENDOR_LIBRARY_NAME=nvidia
	# Gazebo's headless rendering goes through EGL rather than GLX.
	local egl=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
	[ -f "$egl" ] && export __EGL_VENDOR_LIBRARY_FILENAMES="$egl"
	echo "Rendering on $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) (GPU=igpu to keep the integrated GPU)"
}
