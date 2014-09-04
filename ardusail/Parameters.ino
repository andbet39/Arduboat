/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  ArduPlane parameter definitions
 *
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v,&g.v, def}
 
const AS_Param::Info var_info[]  = {
   
    GSCALAR(sysid_this_mav,       "FORMAT_VERSION", 100),
    GSCALAR(rudder_pid_p,         "RUDDER_PID_P", 0.3),
    GSCALAR(rudder_pid_i,         "RUDDER_PID_I", 0.2),
    GSCALAR(rudder_pid_d,         "RUDDER_PID_D", 0.5)
 
};
