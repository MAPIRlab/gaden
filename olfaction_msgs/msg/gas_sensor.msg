# Generic gas sensor msg

# TECHNOLOGY -------------------------------------------------------------------
uint8 TECH_UNKNOWN   = 0
uint8 TECH_MOX       = 1
uint8 TECH_AEC       = 2
uint8 TECH_EQ        = 50
uint8 TECH_PID       = 51
uint8 TECH_SAW       = 52
uint8 TECH_TEMP      = 100
uint8 TECH_HUMIDITY  = 101
uint8 TECH_NOT_VALID = 255


# MANUFACTURER -----------------------------------------------------------------
uint8 MANU_UNKNOWN   = 0
uint8 MANU_FIGARO    = 1
uint8 MANU_ALPHASENSE= 2
uint8 MANU_SGX       = 3
uint8 MANU_RAE       = 50
uint8 MANU_HANWEI    = 51
uint8 MANU_NOT_VALID = 255



# MPN --------------------------------------------------------------------------
uint8 MPN_UNKNOWN    = 0
uint8 MPN_TGS2620    = 50
uint8 MPN_TGS2600    = 51
uint8 MPN_TGS2611    = 52
uint8 MPN_TGS2610    = 53
uint8 MPN_TGS2612    = 54
uint8 MPN_MINIRAELITE= 70
uint8 MPN_NOT_VALID  = 255



# UNITS ------------------------------------------------------------------------
uint8 UNITS_UNKNOWN          = 0
uint8 UNITS_VOLT             = 1
uint8 UNITS_AMP              = 2
uint8 UNITS_PPM              = 3
uint8 UNITS_PPB              = 4
uint8 UNITS_OHM              = 5
uint8 UNITS_CENTIGRADE       = 100
uint8 UNITS_RELATIVEHUMIDITY = 101
uint8 UNITS_NOT_VALID        = 255



# MESSAGE DEFINITION -----------------------------------------------------------

Header header       # timestamp and frame_id

#Info
uint8 technology
uint8 manufacturer
uint8 mpn

#Measurement
float64 raw
uint8 raw_units     # Measurement units (e.g. ppm, volts, ohm)
float64 raw_air     # Sensor output for clean air
float64 calib_A     # Calibration constant. Its exact meaning depends on the sensor
float64 calib_B     # Calibration constant. Its exact meaning depends on the sensor

