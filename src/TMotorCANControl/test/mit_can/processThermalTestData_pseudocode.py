def processThermalTestData_pseudocode(voltage_data,current_data,temp_data, time_data)


    T_ambient = 23 # Ambient temperature measured during trials
    V = voltage_data # Line-to-Line Voltage from DMM
    I = current_data # Commanded Current

    T_c = (temp_data - 32)*5/9 # Changing Fahrenheit to Celcius
    t = time_data # time in minutes from log


    R_LL