#!/usr/bin/env python
# coding: utf-8

# In[4]:


get_ipython().system('pip install pyserial')


# In[1]:


import serial
import numpy as np

data = []
ser = serial.Serial('COM4', 9600)
ser.flushInput()

while True:
    try:
        serial_data = ser.readline()
        try:
            # Convert byte data to string as below
            serial_data = serial_data.decode('utf-8').strip()
            if serial_data == "===":
                print("All data has been received.")
                data = np.array(data)
                np.savetxt('data_log.txt', data)
                break

            # Iterate over each item in array and extract value
            serial_data = serial_data.split()
            raw_data = [0 for x in range(len(serial_data))]
            for i in range(len(serial_data)):
                raw_data[i] = float(serial_data[i])

            # Append data without duplication check
            data.append(raw_data)
        except Exception as e:
            print(f"Error processing data: {e}")
            continue
    except Exception as e:
        print(f"Error reading from serial: {e}")
        break
        