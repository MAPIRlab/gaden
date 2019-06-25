# Generic array of sensors (gas, temp, RH). This is the common msg format for electronic noses.

Header header			# timestamp and frame_id
gas_sensor[] sensors    # the sensors in the array (see gas_sensor msg)
