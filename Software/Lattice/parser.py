class ParamType:
    LETTER = 0
    INTEGER = 1
    FLOAT = 2

class Parser:
    def __init__(self):
        self.triggers = {}
        return

    def register_trigger(self, name, callback, params):
        # Create trigger and then append it.
        self.triggers[name] = {"callback": callback, "params": params}

    def feed_line(self, line):
        line_fields = line.split(' ')

        # If trigger not found, return error message.
        if line_fields[0] not in self.triggers:
            print("Trigger " + line_fields[0] + " not found")
            return False

        # Parse trigger and erase the processed field.
        trigger = self.triggers[line_fields[0]]
        line_fields.pop(0)

        # Parse trigger parameters.
        parsed_params = {}
        for field in line_fields:
            pkey = field[0]
            value = field[1:]
            if pkey in trigger["params"]:
                if trigger["params"][pkey] == ParamType.LETTER:
                    parsed_params[pkey] = value[0]
                if trigger["params"][pkey] == ParamType.INTEGER:
                    parsed_params[pkey] = int(value)
                if trigger["params"][pkey] == ParamType.FLOAT:
                    parsed_params[pkey] = float(value)

        trigger["callback"](parsed_params)
