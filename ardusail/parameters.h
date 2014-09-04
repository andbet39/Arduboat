// Global parameter class.
//
class Parameters {
public:


    //////////////////////////////////////////////////////////////////
    // STOP!!! DO NOT CHANGE THIS VALUE UNTIL YOU FULLY UNDERSTAND THE
    // COMMENTS ABOVE. IF UNSURE, ASK ANOTHER DEVELOPER!!!
    static const uint16_t k_format_version = 13;
    //////////////////////////////////////////////////////////////////


    // The parameter software_type is set up solely for ground station use
    // and identifies the software type (eg ArduPilotMega versus ArduCopterMega)
    // GCS will interpret values 0-9 as ArduPilotMega.  Developers may use
    // values within that range to identify different branches.
    //
    static const uint16_t k_software_type = 0;          // 0 for APM trunk

    enum {
        // Layout version number, always key zero.
        //
        k_param_sysid_this_mav = 0,
        k_param_rudder_pid_p,
        k_param_rudder_pid_i,
        k_param_rudder_pid_d
        
    };
    
  
    // Telemetry control
    //
    AS_Int32 sysid_this_mav;
    AS_Float rudder_pid_p;
    AS_Float rudder_pid_i;
    AS_Float rudder_pid_d;
 
};


extern const AS_Param::Info var_info[];

