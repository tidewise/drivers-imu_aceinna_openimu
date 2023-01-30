# imu_aceinna_openimu

Driver for the [AceINNA OpenIMU](https://www.aceinna.com/openimu)

This driver implements the OpenIMU protocol as provided by the default apps.
However, it uses some protocol extensions that have been implemented on top
of the AceINNA-distributed INS app. The whole protocol is documented in
[Protocol.md](Protocol.md).

The driver will by default refuse to open a device that is not running this
firmware. Copies of the firmware are available in firmwares/ as well as on
GitHub (https://github.com/tidewise/firmwares-imu_aceinna_openimu)

While it is part of the Rock project, this driver is pure CMake and C++ code. It can
be reused on non-Rock environments provided that you also install its dependencies.

The protocol documentation is based on the AceINNA Python library

## Magnetic Calibration

The IMU has a built-in 2D hard/soft iron correction parameters. The E4 message
outputs the corrected measurements.

To calibrate the mags, first set the parameters to zero. You need to save the
config and reset the unit to apply.

```
imu_aceinna_openimu_ctl URL reset # to avoid saving temporary configuration changes
imu_aceinna_openimu_ctl URL set hard-iron-x 0
imu_aceinna_openimu_ctl URL set hard-iron-y 0
imu_aceinna_openimu_ctl URL set soft-iron-angle 0
imu_aceinna_openimu_ctl URL set soft-iron-ratio 1
imu_aceinna_openimu_ctl URL save-config
imu_aceinna_openimu_ctl URL reset # to apply the changes
```

Then do a few full point turns, as far away as possible from buildings and other
possible sources of disturbance (AND do not move too much in X/Y to avoid being
affected by nearby iron). Log the magnetic field information reported by
the IMU.

Then, you have to fit an ellipse. Assuming you have a Rock system using the
imu_aceinna_openimu component based on this driver, convert first the
`magnetic_info` stream to CSV with

```
pocolog LOGFILE -s TASKNAME.magnetic_info --fields magnetometers --csv > uncorrected_mags.txt
```

and then in R, source the magnetic_calibration.r script and do the fit with

```
fit <- magcal("uncorrected_mags.txt")
```

To visualize the raw data and the fit, now do

```
plot(fit$raw_x, fit$raw_y)
lines(fit$fitpoints, col="blue")
```

To get the ellipse parameters,

```
fit$fit
```

### IMU Configuration

The R script returns
- hard iron center in X and Y, which can be provided as-is to the configuration
  parameters.  Note that this is possible because the scripts are written for an
  IMU mounted upwards, as the motion box is.
- soft iron angle can be provided as-is
- soft iron ratio is major / minor

You **must** reset the IMU after changing these parameters. Do

```
<change parameters>
imu_aceinna_openimu_ctl save-config
imu_aceinna_openimu_ctl reset
```

## License

3-clause BSD License

Copyright 2019 (c) TideWise Ltda

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
