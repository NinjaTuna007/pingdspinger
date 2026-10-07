# The 3DSS-DX head's own nadir depth estimate, from the proprietary $PDNDE
# sentence it emits once per ping (source 0, port 99 = internal).
#
#   $PDNDE,<nadir_depth>,<field2>,<field3>,<field4>*cc
#
# Verified (Örebro, 2026-10-07): <nadir_depth> matches the median z of the
# bathymetry points for the same ping to within 0.1 m, negative = down in the
# sonar frame. The remaining fields are undocumented and passed through raw.

std_msgs/Header header
uint64 ping_number

float32 altitude        # m above the seabed at nadir, positive (= -nadir_depth)
float32 nadir_depth     # m, raw first field, negative = below the sonar

float32 field2          # raw, meaning unknown (seen: -0.63 Örebro, -0.06 Askö)
int32 field3            # raw, meaning unknown (seen: 41-84)
int32 field4            # raw, meaning unknown (seen: 53-275)
