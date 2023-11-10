# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

"""
def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    #print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
"""

import socket
import time
import openpyxl

# Configuration
# HOST = '172.20.10.7'
HOST = '192.168.254.7'  # Listen to interfaces
PORT = 8888           # Port to listen on
BUFFER_SIZE = 1024 * 10  # Fixed buffer size 10KB

# Create a new workbook and select the active worksheet
wb = openpyxl.Workbook()
ws = wb.active


def save_to_excel(row_data, row_num):
    # Write binary data to the specified row
    ws.cell(row = row_num, column = 1).value = str(row_data)
    wb.save('received_data.xlsx')


def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Listening on {HOST}:{PORT}...")

        row_num = 1
        while True:  # Infinite loop to keep the server running
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                data = conn.recv(BUFFER_SIZE)
                # Write raw data to excel
                save_to_excel(data, row_num)
                row_num += 1

            #print(f"Connection from {addr} closed.")

                # print("Received:", data.decode('utf-8', errors='ignore'))



if __name__ == "__main__":
    main()


if __name__ == '__main__':
    print('Work done!')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
