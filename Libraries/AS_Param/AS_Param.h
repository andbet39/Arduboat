#include <stdint.h>

#ifndef AS_Param_h
#define AS_Param_h


#include <avr/eeprom.h>

#define MAX_PARAM_NAME_SIZE 16
enum as_var_type
{
    AS_PTYPE_NONE,
    AS_PTYPE_FLOAT,
	AS_PTYPE_INT32,
	AS_PTYPE_UINT32,
    AS_PTYPE_INT8,
    AS_PTYPE_UINT8,
    AS_PTYPE_INT16
};


class AS_Param {
public:


struct Info {   
	uint8_t type;
    const char param_name[MAX_PARAM_NAME_SIZE];
    uint8_t key; 
    void * ptr;
     const float value;

};


  

    AS_Param(){}

    AS_Param(const struct Info *info, uint16_t eeprom_size) {
        _eeprom_size = eeprom_size;
        _var_info = info;

        uint16_t i;
        for (i=0; info[i].type != AS_PTYPE_NONE; i++) ;
        _num_vars = i;
    }
    
    bool save(bool force_save=false);
    
    static void set_value(enum as_var_type type, void *ptr, float value);
    
	static void setup_sketch_defaults(void);
    
   const struct Info * find_var_info()const;

    void copy_name_token(char *buffer, uint8_t bufferSize) const;

    static bool load_all(void);
     bool load(void);
    static void erase_all(void);



    float cast_to_float(enum as_var_type type)const;
    
    static uint8_t count(){
        return _num_vars;
    }

     
    static AS_Param * getByIndex(uint16_t idx,enum as_var_type *p_type);
    static AS_Param * find(const char *name, enum as_var_type *p_type);




private:


    struct EEPROM_header {
        uint8_t magic[2];
        uint8_t revision;
        uint8_t spare;
    };
     struct Param_header {
        uint32_t key : 8;
        uint32_t type : 6;
    };

    static const uint8_t        _sentinal_key   = 0xFF;
    static const uint8_t        _sentinal_type  = 0x3F;


    static void                 write_sentinal(uint16_t ofs);
    static bool                 scan(
                                    const struct Param_header *phdr,
                                    uint16_t *pofs);
    static void                 eeprom_write_check(
                                    const void *ptr,
                                    uint16_t ofs,
                                    uint8_t size);
    
  static const struct Info *  find_by_header(
                                    struct Param_header phdr,
                                    void **ptr);
    static uint8_t type_size(enum as_var_type type);
     
     static uint16_t             _eeprom_size;
     static uint8_t              _num_vars;
     static const struct Info *  _var_info;

        // values filled into the EEPROM header
    static const uint8_t        k_EEPROM_magic0      = 0x50;
    static const uint8_t        k_EEPROM_magic1      = 0x41; ///< "AP"
    static const uint8_t        k_EEPROM_revision    = 6; ///< current format revision
};




template<typename T, as_var_type PT>
class AS_ParamT : public AS_Param
{
public:
    static const as_var_type        vtype = PT;

    /// Value getter
    ///
    const T &get(void) const {
        return _value;
    }

    /// Value setter
    ///
    void set(const T &v) {
        _value = v;
    }

    /// Combined set and save
    ///
    bool set_and_save(const T &v) {
        bool force = (_value != v);
        set(v);
        return save(force);
    }

    /// Combined set and save, but only does the save if the value if
    /// different from the current ram value, thus saving us a
    /// scan(). This should only be used where we have not set() the
    /// value separately, as otherwise the value in EEPROM won't be
    /// updated correctly.
    bool set_and_save_ifchanged(const T &v) {
        if (v == _value) {
            return true;
        }
        set(v);
        return save(true);
    }

    /// Conversion to T returns a reference to the value.
    ///
    /// This allows the class to be used in many situations where the value would be legal.
    ///
    operator const T &() const {
        return _value;
    }

    /// Copy assignment from T is equivalent to ::set.
    ///
    AS_ParamT<T,PT>& operator= (const T &v) {
        _value = v;
        return *this;
    }

    /// bit ops on parameters
    ///
    AS_ParamT<T,PT>& operator |=(const T &v) {
        _value |= v;
        return *this;
    }

    AS_ParamT<T,PT>& operator &=(const T &v) {
        _value &= v;
        return *this;
    }

    AS_ParamT<T,PT>& operator +=(const T &v) {
        _value += v;
        return *this;
    }

    AS_ParamT<T,PT>& operator -=(const T &v) {
        _value -= v;
        return *this;
    }

    /// AS_ParamT types can implement AP_Param::cast_to_float
    ///
    float cast_to_float(void) const {
        return (float)_value;
    }

protected:
    T _value;
};






#define AS_PARAMDEF(_t, _suffix, _pt)   typedef AS_ParamT<_t, _pt> AS_ ## _suffix;
AS_PARAMDEF(float, Float, AS_PTYPE_FLOAT);    // defines AP_Float
AS_PARAMDEF(int32_t, Int32, AS_PTYPE_INT32);     // defines AP_Int8
AS_PARAMDEF(uint32_t, Int32U, AS_PTYPE_UINT32);  // defines AP_Int32
AS_PARAMDEF(int8_t, Int8, AS_PTYPE_INT8);  // defines AP_Int32
AS_PARAMDEF(uint8_t, Int8U, AS_PTYPE_UINT8);  // defines AP_Int32
AS_PARAMDEF(int16_t, Int16, AS_PTYPE_INT16);  // defines AP_Int32



#endif
