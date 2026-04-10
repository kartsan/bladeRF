set_global_assignment -entity "altchip_id" -library "lib_altchip_id" -name IP_TOOL_NAME "altchip_id"
set_global_assignment -entity "altchip_id" -library "lib_altchip_id" -name IP_TOOL_VERSION "20.1"
set_global_assignment -entity "altchip_id" -library "lib_altchip_id" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_altchip_id" -name SPD_FILE [file join $::quartus(sip_path) "altchip_id.spd"]

set_global_assignment -library "lib_altchip_id" -name MISC_FILE [file join $::quartus(sip_path) "altchip_id_sim/altchip_id.vhd"]
set_global_assignment -library "lib_altchip_id" -name MISC_FILE [file join $::quartus(sip_path) "altchip_id_sim/mentor/altchip_id.v"]
set_global_assignment -library "lib_altchip_id" -name MISC_FILE [file join $::quartus(sip_path) "altchip_id_sim/cadence/altchip_id.v"]
set_global_assignment -library "lib_altchip_id" -name MISC_FILE [file join $::quartus(sip_path) "altchip_id_sim/synopsys/altchip_id.v"]
set_global_assignment -library "lib_altchip_id" -name MISC_FILE [file join $::quartus(sip_path) "altchip_id_sim/aldec/altchip_id.v"]
