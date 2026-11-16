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
        set_speed = float(input("Consigne de vitesse : "))
        message_speed = round(set_speed * 10) + 5000
        gain = float(input("Gain Proportionel : "))
        message_gain = round(gain*100)
        gain_i = float(input("Gain Integral : "))
        message_gain_i = round(gain_i*10000)
        message = f"{boucle}{message_speed:07d}{message_gain:05d}{message_gain_i:06d}"
    print(message)
    return message
    

def motor_measure(message, motor_serial):
    motor_serial.write(message.encode('utf-8'))
#    time.sleep(0.05)
#    if motor_serial.in_waiting <= 0:
#        pass
# --- #    
    data_received = ''
    start_time = time.time()
    while True:
        line = motor_serial.read_until(b'>').decode('utf-8')
        if not line:
            break
        data_received += line
        # Stop après 4 s max pour éviter blocage
        if time.time() - start_time > 5:
            break
#            print("No data to read")
#            return np.zeros((1, 3))

#    if motor_serial.in_waiting > 0:
#        data_received=''
#       while motor_serial.inWaiting():
#            data_received+=motor_serial.read(motor_serial.inWaiting()).decode('utf-8')    #read the contents of the buffer
#            time.sleep(0.05)     #depending on your hardware, it can take time to refill the buffer
        # data_received = motor_serial.readline().decode('utf-8')

    print(data_received)
    # --- #
    # This finds all content between < and >, ignoring any junk outside them
    data_separated = re.findall(r'<(.*?)>', data_received)

    print(data_separated)
    L = len(data_separated)
    print(f"Datalength : {L}")

    data = np.zeros((L, 5))
    for k in range(L):
        item = data_separated[k]
        # item is now "1000/250/12.5/12.5/0"
        temp = list(map(float, item.split('/')))
        if len(temp) == 5:
            data[k, :] = temp
    print(data)
    return data
 

def motor_calcul(data, moteur):
    time_data = data[:, 0] / 1e3
    input_data = data[:, 1] - 500
    speed_data = (data[:, 2] - 500) / 100
    setPoint_data = (data[:, 3] - 500) / 100
    Integral_data = (data[:, 4] - 5000)

    time_limit = 100
    speed_data[time_data[:] < time_limit] = 0
    return np.column_stack((time_data, input_data, speed_data,setPoint_data,Integral_data))


def plot_data(data):

# Affichage des résultats
    plt.figure(figsize=(10, 6))

    plt.subplot(211)
    plt.plot(data[:, 0], data[:, 2], linewidth=1, color='blue')
    plt.xlabel('temps (ms)', fontsize=12, color='blue')
    plt.ylabel('vitesse (tr/min)', fontsize=12, color='blue')

    plt.subplot(212)
    plt.plot(data[:, 0], data[:, 1], linewidth=1, color='blue')
    plt.xlabel('temps (ms)', fontsize=12, color='blue')
    plt.ylabel('u(t)', fontsize=12, color='blue')

    plt.tight_layout()
    plt.show()


# ------------------------  #
# --- Main Program Logic -- #
# ------------------------  #

if __name__ == "__main__":
    available_ports = list_serial_ports()
    
    if not available_ports:
        raise Exception("No available serial ports found.")
    
    selected_port = select_port(available_ports)
    motor_serial = serial.Serial(selected_port, baudrate)
    motor_serial.timeout = 5
#    while motor_serial.in_waiting > 0:
#        motor_serial.read()
    motor_serial.reset_input_buffer()
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