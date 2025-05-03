set(srcs 
    "i2c_ht16k33_main.c" 
    )

set(includes 
    "." 
    )

idf_component_register( 
    SRCS ${srcs} 
    INCLUDE_DIRS ${includes} 
    )
