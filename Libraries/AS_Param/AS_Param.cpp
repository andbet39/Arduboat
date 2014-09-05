
#include <Arduino.h>
#include "AS_Param.h"


uint16_t AS_Param::_eeprom_size;
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


void AS_Param::copy_name_token( char *buffer, uint8_t buffer_size) const
{
    
   const struct AS_Param::Info * info = find_var_info();
    if (info == 0) {
        *buffer = 0;
        return;
    }
    strncpy(buffer, info->param_name, buffer_size);
    
    
}



const struct AS_Param::Info *AS_Param::find_var_info() const
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
    case AS_PTYPE_INT8:
        return ((AS_Int8 *)this)->cast_to_float();
    case AS_PTYPE_UINT8:
        return ((AS_Int8U *)this)->cast_to_float();
    case AS_PTYPE_INT16:
        return ((AS_Int16 *)this)->cast_to_float();
    default:
        return 0;
    }
}

 

AS_Param * AS_Param::getByIndex(uint16_t idx,enum as_var_type *p_type){
    
    uint8_t type = _var_info[idx].type;

    *p_type = (enum as_var_type)type;

    return (AS_Param *)_var_info[idx].ptr;
}


AS_Param *
AS_Param::find(const char *name, enum as_var_type *ptype)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        if (strcasecmp(name, _var_info[i].param_name) == 0) {
            *ptype = (enum as_var_type)type;
            return (AS_Param *)_var_info[i].ptr;
        }
    }
    return NULL;
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
    case AS_PTYPE_INT8:
        return 1;
    case AS_PTYPE_UINT8:
        return 1;
    case AS_PTYPE_INT16:
        return 2;
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
    case AS_PTYPE_INT8:
        ((AS_Int8 *)ptr)->set(value);
        break;
    case AS_PTYPE_INT16:
        ((AS_Int16 *)ptr)->set(value);
        break;
    case AS_PTYPE_UINT8:
        ((AS_Int8U *)ptr)->set(value);
        break;
  
    default:
        break;
    }
}


bool AS_Param::save(bool force_save)
{

    uint8_t idx;
    const struct AS_Param::Info *info = find_var_info();
    const AS_Param *ap;

    if (info == NULL) {
        // we don't have any info on how to store it
        return false;
    }

    struct Param_header phdr;

  
    phdr.type = info->type;
    phdr.key  = info->key;

    ap = this;
  
    if (idx != 0) {
        ap = (const AS_Param *)((uintptr_t)ap) - (idx*sizeof(float));
    }

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (scan(&phdr, &ofs)) {
        // found an existing copy of the variable
        eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum as_var_type)phdr.type));
        return true;
    }
    if (ofs == (uint16_t) ~0) {
        return false;
    }

        float v1 = cast_to_float((enum as_var_type)phdr.type);
        float v2;
        
        v2 =  info->value;
        
        if (v1 == v2 && !force_save) {
            return true;
        }
        if (phdr.type != AS_PTYPE_INT32 &&
            (fabs(v1-v2) < 0.0001f*fabs(v1))) {
            // for other than 32 bit integers, we accept values within
            // 0.01 percent of the current value as being the same
            return true;
        }
    

    if (ofs+type_size((enum as_var_type)phdr.type)+2*sizeof(phdr) >= _eeprom_size) {
        // we are out of room for saving variables
        Serial.print("EEPROM full");
        return false;
    }

    // write a new sentinal, then the data, then the header
    write_sentinal(ofs + sizeof(phdr) + type_size((enum as_var_type)phdr.type));
    eeprom_write_check(ap, ofs+sizeof(phdr), type_size((enum as_var_type)phdr.type));
    eeprom_write_check(&phdr, ofs, sizeof(phdr));

        Serial.print("Write param done\n");

    return true;
}
 

bool AS_Param::scan(const AS_Param::Param_header *target, uint16_t *pofs)
{
    struct Param_header phdr;
    uint16_t ofs = sizeof(AS_Param::EEPROM_header);
    while (ofs < _eeprom_size) {
        eeprom_read_block(&phdr, (void*)ofs, sizeof(phdr));
        if (phdr.type == target->type &&
            phdr.key == target->key) {
            // found it
            *pofs = ofs;
            return true;
        }
        // note that this is an ||, not an &&, as this makes us more
        // robust to power off while adding a variable to EEPROM
        if (phdr.type == _sentinal_type ||
            phdr.key == _sentinal_key) {
            // we've reached the sentinal
            *pofs = ofs;
            return false;
        }
        ofs += type_size((enum as_var_type)phdr.type) + sizeof(phdr);
    }
    *pofs = 0xffff;
    //serialDebug("scan past end of eeprom");
    return false;
}


// write to EEPROM
void AS_Param::eeprom_write_check(const void *ptr, uint16_t ofs, uint8_t size)
{
    eeprom_write_block(ptr, (void*)ofs, size);
}

// write a sentinal value at the given offset
void AS_Param::write_sentinal(uint16_t ofs)
{
    struct Param_header phdr;
    phdr.type = _sentinal_type;
    phdr.key  = _sentinal_key;
    eeprom_write_check(&phdr, ofs, sizeof(phdr));
    Serial.print("Write sentinal\n");
}



bool AS_Param::load(void)
{
     uint8_t idx;
    const struct AS_Param::Info *info = find_var_info();
    if (info == NULL) {
        // we don't have any info on how to load it
        return false;
    }

    struct Param_header phdr;

 
    phdr.type = info->type;

    phdr.key  = info->key;

    // scan EEPROM to find the right location
    uint16_t ofs;
    if (!scan(&phdr, &ofs)) {
         set_value((enum as_var_type)phdr.type, (void*) info->ptr,  info->value);

        return false;
    }

    
    AS_Param *ap;
    ap = this;
    if (idx != 0) {
        ap = (AS_Param *)((uintptr_t)ap) - (idx*sizeof(float));
    }

    // found it
    eeprom_read_block(ap,(void*)(ofs+sizeof(phdr)), type_size((enum as_var_type)phdr.type));
    return true;
}



bool AS_Param::load_all(void)
{
    struct Param_header phdr;
    uint16_t ofs = sizeof(AS_Param::EEPROM_header);

    while (ofs < _eeprom_size) {
        eeprom_read_block(&phdr, (void*) ofs, sizeof(phdr));
        // note that this is an || not an && for robustness
        // against power off while adding a variable
        if (phdr.type == _sentinal_type ||
            phdr.key == _sentinal_key) {
            // we've reached the sentinal
            return true;
        }

        const struct AS_Param::Info *info;
        void *ptr;

        info = find_by_header(phdr, &ptr);
        if (info != NULL) {
            eeprom_read_block(ptr, (void*)(ofs+sizeof(phdr)), type_size((enum as_var_type)phdr.type));
        }

        ofs += type_size((enum as_var_type)phdr.type) + sizeof(phdr);
    }

    // we didn't find the sentinal
    Serial.print("no sentinal in load_all");
    return false;
}



// find the info structure given a header
// return the Info structure and a pointer to the variables storage
const struct AS_Param::Info *AS_Param::find_by_header(struct Param_header phdr, void **ptr)
{
    // loop over all named variables
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = _var_info[i].type;
        uint8_t key = _var_info[i].key;
        if (key != phdr.key) {
            // not the right key
            continue;
        }
           *ptr = (void*)_var_info[i].ptr;
            return &_var_info[i];
    }
    return NULL;
}


void AS_Param::erase_all(void)
{
    struct EEPROM_header hdr;

    Serial.print("erase_all");

    // write the header
    hdr.magic[0] = k_EEPROM_magic0;
    hdr.magic[1] = k_EEPROM_magic1;
    hdr.revision = k_EEPROM_revision;
    hdr.spare    = 0;
    eeprom_write_check(&hdr, 0, sizeof(hdr));

    // add a sentinal directly after the header
    write_sentinal(sizeof(struct EEPROM_header));
}