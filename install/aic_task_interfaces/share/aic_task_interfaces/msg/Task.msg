# A unique ID for the cable insertion task.
string id

# The type of cable which should be inserted.
string cable_type

# The name of the cable which should be inserted, e.g., "sfp_sc".
string cable_name

# The type of plug at the end of the cable which should be inserted, e.g., "sfp".
string plug_type

# The name of plug at the end of the cable which should be inserted, e.g., "sfp_module".
string plug_name

# The type of port on the module side into which the plug should be inserted, e.g., "sfp".
string port_type

# The name of the port on the module side into which the plug should be inserted, e.g., "sfp_port_0".
string port_name

# The name of the component or module on which the port exists, e.g., "nic_card_0"
string target_module_name

# The number of seconds from the moment this request is received by when the task must complete.
# todo(yadunund): should this be a system clock timestamp instead.
uint64 time_limit
