
#include "AS_Param.h"
// number of rows in the _var_info[] table
uint8_t AS_Param::_num_vars;

// storage and naming information about all types that can be saved
const AS_Param::Info *AS_Param::_var_info;



// load default values for all scalars in a sketch. This does not
// recurse into sub-objects
void AS_Param::setup_sketch_defaults(void)
{

	 for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        void *ptr = (void*)_var_info[i].ptr;
        set_value((enum as_var_type)type, ptr, _var_info[i].value);
        
    }
}

bool AS_Param::save(bool force_save){

}





void AS_Param::set_value(enum as_var_type type, void *ptr, float value)
{
    switch (type) {
    case AS_PTYPE_INT32:
        ((AS_Int32 *)ptr)->set(value);
        break;
    case AS_PTYPE_FLOAT:
        ((AS_Float *)ptr)->set(value);
        break;
    case AS_PTYPE_UINT32:
        ((AS_Int32U *)ptr)->set(value);
        break;
  
    default:
        break;
    }
}
 