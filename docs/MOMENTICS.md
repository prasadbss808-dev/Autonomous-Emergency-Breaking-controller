# Running the whole system from QNX Momentics IDE

`aeb_master` is one program that starts everything on the Pi, in this order:
radar UART → IMU pins → camera service → dashboard → camera detection → guard → AEB.

The AEB then runs inside the Momentics console. You see the status lines and the
`OBJECT DETECTED` events there, and you can type `go`, `stop`, `estop`, `status` and `quit`.
**Terminate** (the red square) stops the AEB safely: it brakes, then disables the drivers.

## One-time setup

1. **Deploy the files** (terminal, laptop): `make deploy-all OPENCV_ROOT=~/qnx_workspace/staging TARGET=qnxuser@<pi-ip>`
2. **Import the project.** *File → Import → C/C++ → Existing Code as Makefile Project*.
   Choose the `qnx_aeb` folder and the *QNX GCC* toolchain, then *Finish*.
3. **Build.** Right-click the project → *Build Project*. This runs `make` and produces `build/aeb_master`.
   Source the QNX environment first if Momentics did not do it.
4. **Target.** *Window → Show View → QNX Targets*. Add the Pi (`172.16.6.76`) and log in as **root**.
5. **Launch configuration.** *Run → Run Configurations → C/C++ QNX Application → New*:
   - *Main*: Project `qnx_aeb`, C/C++ Application `build/aeb_master`
   - *Target*: the Pi
   - *Upload / Remote*: remote directory **/data/home/qnxuser/aeb**
   - *Arguments* (optional): `soft` (software PWM), `novision`, `sim` (simulator camera), `nodash`
   - *Run*.

The dashboard address is printed in the console: **http://<pi-ip>:8080**.

## If Momentics cannot log in as root
The AEB needs root (GPIO, UART driver, real-time priorities). If the target connection
uses `qnxuser`, give the launcher root rights once on the Pi (as root):
```sh
cd /data/home/qnxuser/aeb && chown root aeb_master && chmod u+s aeb_master
```
Then turn off the binary upload in the launch configuration (use the remote binary as is), because
a new upload replaces the file and removes the root bit. Alternatively, start it from the Momentics
*Target Terminal* with `su` and `./aeb_master`.
