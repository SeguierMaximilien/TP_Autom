import time
import numpy as np
import re
import matplotlib.pyplot as plt
import serial.tools.list_ports
import serial

# --------------------------- #
# ---   PARAMETERS   -------- #
baudrate = 115200  # Must match the Baudrate defined in the arduino program.
MOTEUR = {
    'Data_Size': 10,
    'Encoder_Resolution': 48 * 172  # 172:1
}
# --------------------------- #
# --------------------------- #

def list_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]

def select_port(available_ports):
    print("Selectionnez un port:")
    for i, port in enumerate(available_ports):
        print(f"{i + 1}: {port}")
    selected = int(input("Entrez le numéro du port: ")) - 1
    return available_ports[selected]

def motor_interpret(mode, moteur):
    boucle = mode - 1
    if boucle == 0:
        ppm = input('u(t) in [-255;255]: ')
        ppm = 500 + int(ppm)
        ppm = min(755, max(245, ppm))
        message = f"{boucle}{ppm:03d}00000000"
    elif boucle == 1:
        angle = float(input("Consigne d'angle: "))
        gain = float(input("Gain: "))
        angle_to_ticks = round(angle * moteur['Encoder_Resolution'] / 360)
        angle_offset = 1e6  # Should be used in Arduino code 
        code = angle_offset + angle_to_ticks
        message_code = f"{int(code):07d}"
        gain_value = round(gain * 360 / moteur['Encoder_Resolution'] * 1000)
        message_gain = f"{int(gain_value):05d}"
        message = f"{boucle}{message_code}{message_gain}"
    print(message)
    return message

def motor_measure(message, motor_serial):
    motor_serial.write(message.encode('utf-8'))    
#    time.sleep(0.11)
#    if motor_serial.in_waiting <= 0:
#        pass
# --- #    
    data_received=''
    start_time = time.time()
    while True:
        line = motor_serial.read_until(b'>').decode('utf-8')
        # if not line:
        #    break
        data_received += line
        print(time.time() - start_time )
        # Stop après 5 s max pour éviter blocage
        if time.time() - start_time > 5:
            break

    print(data_received)
    # --- #
    # This finds all content between < and >, ignoring any junk outside them
    data_separated = re.findall(r'<(.*?)>', data_received)
    
  #  print(data_separated)
    L = len(data_separated)
    print(f"Datalength : {L}")

    
    data = np.zeros((L, 4))
    for k in range(L):
        item = data_separated[k]
        # item is now "1000/250/12.5/12.5/0"
        temp = list(map(float, item.split('/')))
        if len(temp) == 4:
            data[k, :] = temp
    print(data)
    return data

def motor_calcul(data, moteur):
    time_data = data[:, 0] / 1e3
    input_data = data[:, 1] - 500
    position = (data[:, 2] - 1e6) * 360 / moteur['Encoder_Resolution']
    setPoint = (data[:, 3] - 1e6) * 360 / moteur['Encoder_Resolution']
    # ---- #
    variation_position = np.diff(position)
    variation_time = np.diff(time_data)
    # ---- #
    speed = variation_position * 60000 / (variation_time * 360)
    speed = np.insert(speed, 0, 0)
    # ---- #
    data_converted = np.column_stack((time_data, input_data, position, speed,setPoint))
    data_converted[np.isnan(data_converted)] = 0
    return data_converted

def plot_data(data):
    plt.figure(figsize=(10, 8))
    # --- #
    plt.subplot(311)
    plt.plot(data[:, 0], data[:, 2], linewidth=1, color='blue')
    plt.xlabel('temps (ms)', fontsize=12, color='blue')
    plt.ylabel('position (°)', fontsize=12, color='blue')
    # --- #
    plt.subplot(312)
    plt.plot(data[:, 0], data[:, 3], linewidth=1, color='blue')
    plt.xlabel('temps (ms)', fontsize=12, color='blue')
    plt.ylabel('vitesse (tr/min)', fontsize=12, color='blue')
    # --- #
    plt.subplot(313)
    plt.plot(data[:, 0], data[:, 1], linewidth=1, color='blue')
    plt.xlabel('temps (ms)', fontsize=12, color='blue')
    plt.ylabel('u(t)', fontsize=12, color='blue')
    # --- #
    plt.tight_layout()
    plt.show()

# ------------------------ #
# --- Main Program Logic -- #
# ------------------------ #

if __name__ == "__main__":
    available_ports = list_serial_ports()
    
    if not available_ports:
        raise Exception("No available serial ports found.")
    
    selected_port = select_port(available_ports)
    motor_serial = serial.Serial(selected_port, baudrate)
    motor_serial.timeout = 1
    while motor_serial.in_waiting > 0:
        motor_serial.read()
    print(f"{selected_port} is connected")

    mode = int(input("Mode (1: Boucle ouverte, 2: Boucle fermée): "))
    message = motor_interpret(mode, MOTEUR)

    print("Begin experiment")
    tab = motor_measure(message, motor_serial)
    print("End experiment")

    print(f"{selected_port} is now closed")
    motor_serial.close()

    data = motor_calcul(tab, MOTEUR)
    np.savetxt('Measure.txt', data)

    plot_data(data)