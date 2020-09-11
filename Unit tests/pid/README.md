# cparser
Simple parser for embedded C/C++ applications.

## Usage
Copy cparser folder which includes **src** and **inc** folders to your 
project folder. Include **cparser.h** in the files using the cparser 
functionality.

Interface of the cparser is pretty basic. Just with three
functions application can process trigger(command or command response) strings. 
These functions are,

* void Cp_Register(Cp_Trigger_t *triggers, uint16_t numOfTriggers)
* uint8_t Cp_FeedLine(char *input)
* void Cp_Reset(void)

Cparser has a dynamic structure and the triggered to be parsed are needed to
be "registered" by the application program. Then cparser is good to go; "feed" 
every received line to cparser for parsing process.

### Registering a trigger
**Cp_Register** is used to register a trigger. Trigger is a structure
which includes trigger name, parameters and callback pointer. Trigger name is just
a string, parameters are objects which contain parameter type(letter, integer or real)
and a letter denoting that parameter. Callback pointer is a function pointer which 
will be invoked when the trigger name matched the registered trigger. 
By the use of such a structure different modules in the application program 
can register their own triggers and can work independent of each other. 

### Feeding line of trigger string
In order to cparser process the trigger string, trigger should be given in a
null terminated char array via **Cp_Feedline** command.

### Clearing trigger registry
Sometimes it may be useful to clear trigger registry, for example for testing
purposes. **Cp_Reset** function should be called in order to achieve this.

## Configuration
By changing the constants in the **cparser_config.h** file, cparser can be configured.
These constants are;

* CPARSER_CONFIG_MAX_NUM_OF_PARAMS 
Determines maximum number of parameters which a trigger
can have(default: 5). 
* CPARSER_CONFIG_MAX_NUM_OF_TRIGGERS 
Determines maximum number of triggers to be registered(default: 25).
* CPARSER_CONFIG_MAX_TRIGGER_LENGTH 
Determines maximum trigger name string length except the
null termination (default: 5).

## Documentation
Doxygen generated documentation can be found under doc folder. 
