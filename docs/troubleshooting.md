# Troubleshooting

## Low real-time factor on Gazebo

If your machine has two GPUs (or a CPU with an integrated GPU), OpenGL may be using the *integrated* GPU for rendering, which causes RTF to be very low. To fix this, you may need to manually force it to use the *discrete* GPU.

To check if Open GL is using the discrete GPU, run `glxinfo -B`: it should show the details of your discrete GPU. You can also run `nvidia-smi` and when the AIC sim is launched: `gz sim` should be listed as one of the processes.

If the wrong GPU is selected, run `sudo prime-select nvidia`, log out and log in again. Check `glxinfo -B` again to make sure the discrete GPU has been selected.

You can also check out [Problems with dual Intel and Nvidia GPU systems](https://gazebosim.org/docs/latest/troubleshooting/#problems-with-dual-intel-and-nvidia-gpu-systems).
