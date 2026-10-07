# Sonar settings and system info in effect for a ping (DxParameters +
# DxSystemInfo on the 3DSS-DX wire). Published on sonar/settings with a
# transient-local QoS whenever any value changes, so late subscribers and bag
# recordings get the configuration that the sidescan/bathymetry amplitudes
# were acquired under. Without these (TVG gain, pulse, power, beamwidth) the
# amplitudes of different lines or surveys are not comparable.

std_msgs/Header header
uint64 ping_number              # first ping these settings were seen on

# System
string sonar_id
float32 acoustic_frequency      # Hz
float32 sample_rate             # Hz (sidescan receive sample rate)
float32 maximum_ping_rate       # Hz, for the current range setting
float32 port_transducer_angle   # deg, mounting tilt, positive = downward
float32 starboard_transducer_angle

# Range
float32 range                   # m, range setting
float32 sidescan_bin_spacing    # m, = sound_velocity_bulk / (2 * sample_rate)
float32 port_sidescan_range_resolution        # m, pulse range resolution (NOT bin spacing)
float32 starboard_sidescan_range_resolution
float32 port_sidescan3d_range_resolution      # m, 3D range step
float32 starboard_sidescan3d_range_resolution

# Sound velocity
float32 sound_velocity_bulk     # m/s, operator constant used for all ranges
float32 sound_velocity_face     # m/s, live probe at the transducer face

# Receive gain (TVG): G_dB = constant + linear*R + logarithmic*log10(R)
float32 port_gain_constant
float32 port_gain_linear
float32 port_gain_logarithmic
float32 starboard_gain_constant
float32 starboard_gain_linear
float32 starboard_gain_logarithmic

# Transmit
float32 port_transmit_angle     # deg relative to transducer MRA, positive = downward
uint32 port_transmit_power      # percent of maximum
string port_transmit_beamwidth  # e.g. "Degrees90"
string port_transmit_pulse      # e.g. "Narrowband-10"
float32 starboard_transmit_angle
uint32 starboard_transmit_power
string starboard_transmit_beamwidth
string starboard_transmit_pulse

# Trigger
string trigger_source           # e.g. "Continuous"
float32 trigger_continuous_duty_cycle   # fraction of maximum_ping_rate

# Sidescan (2D) processing
string port_sidescan_mode               # e.g. "Coherent"
string port_sidescan_incoherent_method  # e.g. "EnvelopeRms"
string port_sidescan_coherent_beams     # e.g. "DownAll"
string starboard_sidescan_mode
string starboard_sidescan_incoherent_method
string starboard_sidescan_coherent_beams

# Sidescan-3D (angle estimation) processing
int32 port_sidescan3d_smoothing         # receive samples per angle estimate
float32 port_sidescan3d_tolerance       # non-plane-wave tolerance
float32 port_sidescan3d_threshold       # dB re FS
int32 port_sidescan3d_number_of_angles  # angles per range step
int32 starboard_sidescan3d_smoothing
float32 starboard_sidescan3d_tolerance
float32 starboard_sidescan3d_threshold
int32 starboard_sidescan3d_number_of_angles
