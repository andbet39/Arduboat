
#include <Arduino.h>
#include "AS_Param.h"

// number of rows in the _var_info[] table
uint8_t AS_Param::_num_vars;

// storage and naming information about all types that can be saved
const AS_Param::Info *AS_Param::_var_info;


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

void AS_Param::copy_name_token( char *buffer, uint8_t buffer_size) const
{
    
   const struct AS_Param::Info *info = find_var_info();
    if (info == NULL) {
        *buffer = 0;
        return;
    }
    strncpy(buffer, info->param_name, buffer_size);
    
    
}



const struct AS_Param::Info *AS_Param::find_var_info()
{
    for (uint8_t i=0; i<_num_vars; i++) {
        void *ptr = (void*)_var_info[i].ptr;
        if (ptr==this)
        {
            return &_var_info[i];
        }
            
    }
    return 0;
}



/// cast a variable to a float given its type
float AS_Param::cast_to_float(enum as_var_type type) const
{
    switch (type) {
    case AS_PTYPE_INT32:
        return ((AS_Int32U *)this)->cast_to_float();
    case AS_PTYPE_UINT32:
        return ((AS_Int32 *)this)->cast_to_float();
     case AS_PTYPE_FLOAT:
        return ((AS_Float *)this)->cast_to_float();
    default:
        return 0;
    }
}

 

AS_Param * AS_Param::getByIndex(uint16_t idx,enum as_var_type *p_type){
    
    uint8_t type = _var_info[idx].type;

    //ptype = (enum as_var_type)type;
    return (AS_Param *)_var_info[idx].ptr;
}



// return the storage size for a AP_PARAM_* type
uint8_t AS_Param::type_size(enum as_var_type type)
{
    switch (type) {
    case AS_PTYPE_INT32:
        return 4;
    case AS_PTYPE_UINT32:
        return 4;
    case AS_PTYPE_FLOAT:
        return 4;
    }

    return 0;
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
 