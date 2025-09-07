import socket
import time
import board
import adafruit_icm20x
import numpy as np

i2c = board.I2C()
icm = adafruit_icm20x.ICM20948(i2c)

# HOST IP
PORT = 65432

def main():
    b_gyr = np.array([-0.0159877493129771, 0.0667488533816794, 0.01279019945038168])

    A_acc = np.array([[ 1.018329, 0.002709, -0.004729],
                      [ 0.002709, 1.003068,  0.001185],
                      [-0.004729, 0.001185,  0.991777]])

    b_acc = np.array([0.146510, -0.143817, -0.194597])

    A_mag = np.array([[ 1.293852, -0.069138, -0.037015],
                      [-0.069138,  1.097919, -0.002501],
                      [-0.037015, -0.002501,  1.207242]])

    b_mag = np.array([16.508940, -41.661465, 29.020480])

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
        mx, my, mz = raw_mag
    
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
