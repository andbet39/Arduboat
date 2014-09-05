/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  ArduPlane parameter definitions
 *
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v,&g.v, def}
 
const AS_Param::Info var_info[]  = {
    GSCALAR(format_version,         "FORMAT_VERSION", 0),
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),
    GSCALAR(sysid_this_mav,       "SYSTEM_ID",    100),
    GSCALAR(rudder_pid_p,         "RUDDER_PID_P", 0.3),
    GSCALAR(rudder_pid_i,         "RUDDER_PID_I", 0.2),
    GSCALAR(rudder_pid_d,         "RUDDER_PID_D", 0.5),
    GSCALAR(rudder_dead_zone,      "RUDDER_S_DEAD", 15)
  
};

static void load_parameters(void)
{
 
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        Serial.print("Firmware change: erasing EEPROM...\n");
        AS_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        Serial.print(("done."));
    } else {
         // Load all auto-loaded EEPROM variables
        AS_Param::load_all();
        Serial.print("load_all");
    }
}
