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
