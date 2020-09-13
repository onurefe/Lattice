import json
import time
import serial

def loadFactoryData():
    print("Enter factory configuration file path: ")
    config_file = input()
    print("Enter the serial port which device is connected")
    
    f = open(config_file, "r")
    cfg = json.loads(f.read())
    sport = serial.Serial()
    
    # Search error detection parameters.
    if "error_detection_params" in cfg:
        error_detection_params_found = (("min_horn_impedance" in cfg["error_detection_params"]) and
                                        ("max_horn_impedance" in cfg["error_detection_params"]) and
                                        ("power_tracking_tolerance" in cfg["error_detection_params"]) and
                                        ("frequency_tracking_tolerance" in cfg["error_detection_params"]) and
                                        ("monitoring_period" in cfg["error_detection_params"]) and
                                        ("timeout" in cfg["error_detection_params"]))
    else:
        error_detection_params_found = False

    # Search constraints.
    if "constraints" in cfg:
        constraints_found = (("max_power" in cfg["constraints"]) and
                             ("min_loading" in cfg["constraints"]) and
                             ("max_loading" in cfg["constraints"]))
    else:
        constraints_found = False

    # Search device information.
    if "device_info" in cfg:
        device_info_found = (("device_id" in cfg["device_info"]) and
                             ("version_major" in cfg["device_info"]) and
                             ("version_minor" in cfg["device_info"]))
    else:
        device_info_found = False

    # Search searching parameters.
    if "searching_parameters" in cfg:
        searching_parameters_found = (("normalized_power" in cfg["searching_parameters"]) and
                                      ("steps" in cfg["searching_parameters"]))
    else:
        searching_parameters_found = False

    # Search power tracking PID parameters.
    if "power_tracking_pid" in cfg:
        power_tracking_pid_found = (("kp" in cfg["power_tracking_pid"]) and
                                    ("ki" in cfg["power_tracking_pid"]) and
                                    ("kd" in cfg["power_tracking_pid"]) and
                                    ("tf" in cfg["power_tracking_pid"]))
    else:
        power_tracking_pid_found = False

    # Search frequency tracking PID parameters.
    if "frequency_tracking_pid" in cfg:
        frequency_tracking_pid_found = (("kp" in cfg["frequency_tracking_pid"]) and
                                        ("ki" in cfg["frequency_tracking_pid"]) and
                                        ("kd" in cfg["frequency_tracking_pid"]) and
                                        ("tf" in cfg["frequency_tracking_pid"]))
    else:
        frequency_tracking_pid_found = False

    # Search destination power parameter.
    destination_power_found = "destination_power" in cfg

    # Inform user if the parameters are found successfully.
    if error_detection_params_found:
        print("SUCCESS: Error detection parameters found")
    else:
        print("FAILURE: Error detection parameters are obsolote or incomplete")

    if constraints_found:
        print("SUCCESS: Constraints found")
    else:
        print("FAILURE: Constraints are obsolote or incomplete")

    if device_info_found:
        print("SUCCESS: Device info found")
    else:
        print("FAILURE: Device info are obsolote or incomplete")

    if searching_parameters_found:
        print("SUCCESS: Searching parameters found")
    else:
        print("FAILURE: Searching parameters are obsolete or incomplete")

    if power_tracking_pid_found:
        print("SUCCESS: Power tracking PID parameters found")
    else:
        print("FAILURE: Power tracking PID parameters are obsolete or incomplete")

    if frequency_tracking_pid_found:
        print("SUCCESS: Frequency tracking PID parameters found")
    else:
        print("FAILURE: Frequency tracking PID parameters are obsolete or incomplete")

    if destination_power_found:
        print("SUCCESS: Destination power found")
    else:
        print("FAILURE: Destination power is obsolete")

    # Create commands strings.
    cstrt_s = "cstr-s" + " P" + cfg["constraints"]["max_power"] + \
        " N" + cfg["constraints"]["min_loading"] + \
        " X" + cfg["constraints"]["max_loading"]

    devinf_s = "devinf-s" + " I" + cfg["device_info"]["device_id"] + \
        " J" + cfg["device_info"]["version_major"] + \
        " N" + cfg["device_info"]["version_minor"]

    srcprms_s = "srcpms-s" + " P" + cfg["searching_parameters"]["normalized_power"] + \
                " S" + cfg["searching_parameters"]["steps"]

    errdet_s = "errdet-s" + " N" + cfg["error_detection_params"]["min_horn_impedance"] + \
        " X" + cfg["error_detection_params"]["max_horn_impedance"] + \
        " P" + cfg["error_detection_params"]["power_tracking_tolerance"] + \
        " F" + cfg["error_detection_params"]["frequency_tracking_tolerance"] + \
        " R" + cfg["error_detection_params"]["monitoring_period"] + \
        " T" + cfg["error_detection_params"]["timeout"]

    frpid_s = "frpid-s" + " P" + cfg["frequency_tracking_pid"]["kp"] + \
        " I" + cfg["frequency_tracking_pid"]["ki"] + \
        " D" + cfg["frequency_tracking_pid"]["kd"] + \
        " T" + cfg["frequency_tracking_pid"]["tf"]

    pwpid_s = "pwpid-s" + " P" + cfg["power_tracking_pid"]["kp"] + \
        " I" + cfg["power_tracking_pid"]["ki"] + \
        " D" + cfg["power_tracking_pid"]["kd"] + \
        " T" + cfg["power_tracking_pid"]["tf"]

    power_s = "power-s" + " P" + cfg["destination_power"]

    cmds = [cstrt_s, devinf_s, srcprms_s, errdet_s, frpid_s, pwpid_s, power_s]

    for cmd in cmds:
        print(cmd)
        print("Awaiting response..")
        serial
        time.sleep(0.1)
