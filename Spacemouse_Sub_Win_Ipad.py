#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  2 02:38:10 2024
@author: kathanpatel sub.py
"""
import socket
import json
import time

def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return True, sock
    except Exception as e:
        sock.close()
        return False, None

def disconnectETController(sock):
    if sock:
        sock.close()

def sendCMD(sock, cmd, params=None, id=1):
    if not params:
        params = []
    else:
        params = json.dumps(params)
    sendStr = '{{"method":"{0}","params":{1},"jsonrpc":"2.0","id":{2}}}\n'.format(cmd, params, id)
    try:
        sock.sendall(sendStr.encode('utf-8'))
        ret = sock.recv(1024)
        jdata = json.loads(ret.decode('utf-8'))
        if 'result' in jdata:
            return True, json.loads(jdata['result']), jdata['id']
        elif 'error' in jdata:
            return False, jdata['error'], jdata['id']
        else:
            return False, None, None
    except Exception as e:
        return False, None, None

def set_v(data, robot_speed, omega):
    global current_pose
    
    for i in range(6):
        if data[i] == 1:
            current_pose[i] = robot_speed
        elif data[i] == -1:
            current_pose[i] = -robot_speed
        else:
            current_pose[i] = 0        
    
    return current_pose

def calculate_inverse_kinematics(robot_ip, target_pose):
    P000 = [0, -90, -90, 90, -90, 0]

    conSuc, sock = connectETController(robot_ip)

    if conSuc:
        try:
            suc, result, id = sendCMD(sock, "inverseKinematic", {"targetPose": target_pose, "referencePos": P000})
            if suc:
                return result
            else:
                print("Inverse kinematics failed for this pose:", result)
                return None

        except Exception as e:
            print("Error:", e)
        finally:
            disconnectETController(sock)
    else:
        print("Connection to the robot failed.")
        return None

def Orientation_correct(robot_ip):
    conSuc, sock = connectETController(robot_ip)

    if not conSuc:
        return None

    suc, result, id = sendCMD(sock, "set_servo_status", {"status": 1})
    suc, Current_tcp, id = sendCMD(sock, 'get_tcp_pose', {'coordinate_num': 0, 'tool_num': 0})
    
    points = [Current_tcp[0], Current_tcp[1], Current_tcp[2], -1.57,0,0]
    
    print("This is the desired point in coordinate system" , points)
    
    angle_point = calculate_inverse_kinematics(robot_ip, points)
    
    angle_point[5] = angle_point[5] - 90

    print("Moving to point linearly:", angle_point)

    if angle_point is None:
        print("Error: Target position for linear motion is invalid.")
        return

    suc, result, id = sendCMD(sock, "moveByLine", {
        "targetPos": angle_point,
        "speed_type": 0,
        "speed": 200,
        "cond_type": 0,
        "cond_num": 7,
        "cond_value": 1})

    if not suc:
        print("Error in moveByLine:", result)
        return

    while True:
        suc, result, id = sendCMD(sock, "getRobotState")
        if result == 0:
            break

def main():
    global robot_speed
    global omega
    global current_pose
    global correcting_orientation

    robot_speed = 10
    omega = 10
    current_pose = [0] * 8  # Initialize current_pose array
    final_matrix = [0] * 6
    mode = 0  # Initialize mode
    correcting_orientation = False

    robot_ip = '192.168.1.200'
    conSuc, robot_sock = connectETController(robot_ip)
    if not conSuc:
        print('Failed to connect to the robot.')
        return

    sendCMD(robot_sock, 'set_servo_status', {'status': 1})
    # Set the loop mode to single loop
    suc, result, id = sendCMD(robot_sock, 'setCycleMode', {'cycle_mode': 2})
    suc, result, id = sendCMD(robot_sock, 'setCurrentCoord', {'coord_mode': 1})

    if conSuc:
        suc, result_pose, id = sendCMD(robot_sock, 'get_tcp_pose', {'coordinate_num': 0, 'tool_num': 0})
        print(result_pose)
        suc, result, id = sendCMD(robot_sock, 'setSysVarV', {'addr': 1, 'pose': current_pose})

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = '127.0.0.1'
    port = 12345
    client_socket.connect((host, port))

    last_safety_check = time.time()

    try:
        while True:
            if correcting_orientation:
                time.sleep(0.1)  # Give time for orientation correction
                continue

            data = client_socket.recv(1024)
            if not data:
                decoded_data = [0] * 8
            else:
                try:
                    decoded_data = json.loads(data.decode('utf-8'))
                except json.decoder.JSONDecodeError as e:
                    print("Error decoding JSON:", e)
                    decoded_data = [0] * 8
                    continue  # Skip processing if decoding fails

            for i in range(6):
                if i < 3:
                    if decoded_data[i] > 100:
                        decoded_data[i] = 1
                    elif decoded_data[i] < -100:
                        decoded_data[i] = -1
                    else:
                        decoded_data[i] = 0
                else:
                    if decoded_data[i] > 150:
                        decoded_data[i] = 1
                    elif decoded_data[i] < -150:
                        decoded_data[i] = -1
                    else:
                        decoded_data[i] = 0

            # Perform safety check less frequently (e.g., every 0.5 seconds)
            if time.time() - last_safety_check > 0.5:
                suc, Saftey, id = sendCMD(robot_sock, 'getVirtualOutput', {'addr': 440})
                last_safety_check = time.time()

            suc, Saftey_joint, id = sendCMD(robot_sock, 'getVirtualOutput', {'addr': 528})
            
            if Saftey_joint == 1:
                suc, result, id = sendCMD(robot_sock, "set_servo_status", {"status": 0})
                
            
            
            

            if decoded_data != [0] * 8 and Saftey != 0 and Saftey_joint == 0:
                if decoded_data[6] == 1 and decoded_data[7] == 1 and not correcting_orientation:
                    mode = 1
                    correcting_orientation = True  # Set flag before calling orientation correction
                    print(f"Switched to mode {mode}")
                    Orientation_correct(robot_ip)
                    correcting_orientation = False  # Reset flag after correction
                    mode = 0  # Reset mode after correction

                if decoded_data[6] == 1 and robot_speed > 0 and omega > 0:
                    robot_speed -= 5
                    omega -= 2
                    print('Speed Decreased to:', robot_speed)
                elif decoded_data[7] == 1 and robot_speed < 200 and omega < 150:
                    robot_speed += 5
                    omega += 2
                    print('Speed increased to:', robot_speed)
                else:
                    temp = set_v(decoded_data, robot_speed, omega)
                    final_matrix = temp[:6]
                    final_matrix[0] = final_matrix[0]
                    final_matrix[1] = -final_matrix[1]
                    final_matrix[2] = -final_matrix[2]
                    final_matrix[3] = final_matrix[3]
                    final_matrix[4] = -final_matrix[4]
                    final_matrix[5] = -final_matrix[5]

                    if len(decoded_data) == 8 and not correcting_orientation:
                        print(f'Received: {final_matrix}')
                        suc, result, id = sendCMD(robot_sock, 'moveBySpeedl', {'v': final_matrix, 'acc': 50, 'arot': 10, 't': 0.1})
                        print(suc, result, id)
            else:
                suc, result, id = sendCMD(robot_sock, "stopl", {"acc": 490})
                decoded_data = [0] * 8

    except KeyboardInterrupt:
        print('Client stopped.')
    finally:
        client_socket.close()
        disconnectETController(robot_sock)

if __name__ == '__main__':
    main()
