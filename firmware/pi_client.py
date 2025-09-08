import socket
import time
import board
import adafruit_icm20x
import numpy as np

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c)

#HOST = '192.168.0.2'  # PC's ethernet IP address
HOST = '192.168.0.40'  # PC's wifi IP address
PORT = 65432

def main():
    b_gyr = np.array([-0.0159877493129771, 0.0667488533816794, 0.01279019945038168])

    A_acc = np.array([[ 0.998846, -0.002210, -0.000511],
                      [-0.002210,  0.998428,  0.003091],
                      [-0.000511,  0.003091,  0.992201]])

    b_acc = np.array([-0.038146, -0.153961, -0.150686])

    A_mag = np.array([[1.139215,  0.002256,  0.008569],
                      [0.002256,  1.115183, -0.046443],
                      [0.008569, -0.046443,  1.186528]])

    b_mag = np.array([20.709293, -39.228641, 22.677635])

    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print("Connection successful.")
            break
        except (ConnectionRefusedError, OSError) as e:
            print(f"Connection failed: {e}. Retrying in 2s...")
            time.sleep(2)
    
    while True:
        raw_gyr = np.array(icm.gyro)
        raw_acc = np.array(icm.acceleration)
        raw_mag = np.array(icm.magnetic)

        gyr = raw_gyr - b_gyr
        acc = np.linalg.inv(A_acc) @ (raw_acc - b_acc)
        mag = np.linalg.inv(A_mag) @ (raw_mag - b_mag)

        gx, gy, gz = gyr
        ax, ay, az = acc
        mx, my, mz = mag
    
        msg = f"{gx},{gy},{gz},{ax},{ay},{az},{mx},{my},{mz}\n"

        try:
            s.sendall(msg.encode())
        except (BrokenPipeError, ConnectionResetError):
            print("Connection lost. Reconnecting...")
            s.close()

            while True:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.connect((HOST, PORT))
                    print("Connection successful.")
                    break
                except (ConnectionRefusedError, OSError) as e:
                    print(f"Connection failed: {e}. Retrying in 2s...")
                    time.sleep(2)


if __name__ == "__main__":
    main()
