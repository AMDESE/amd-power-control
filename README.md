# AMD power control

This repository implements AMD EthanolX power control based off of OpenBMC compliant
implementation of power control for x86 servers.
It relies on a number of features to do its job.  It has
several intentional design goals.
1. The BMC should maintain the Host state machine internally, and be able to
   track state changes.
2. The implementation should either give the requested power control result, or
   should log an error on the failure it detected.
3. The BMC should support all the common operations, hard power on/off/cycle,
   soft power on/off/cycle.

Caveats:
This implementation does not currently implement the other sp3 platforms.