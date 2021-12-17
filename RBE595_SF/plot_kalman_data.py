import numpy as np
import os
import pandas as pd

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('seaborn-whitegrid')

def plot_kalman_data( df ):

    print("Plotting Kalman...")
    fig1 = plt.figure()
    pos = fig1.add_subplot(111)
    pos.plot( df["TIME"][1:], df["P_X"][1:], label='X')
    pos.plot( df["TIME"][1:], df["P_Y"][1:], label='Y')
    pos.plot( df["TIME"][1:], df["P_Z"][1:], label='Z')
    pos.set_xlabel('Time (s)')
    pos.set_ylabel('Position (m)')
    pos.legend()
    fig1.show()

    fig2 = plt.figure()
    vel = fig2.add_subplot(111)
    vel.plot( df["TIME"][1:], df["V_X"][1:], label='Xd')
    vel.plot( df["TIME"][1:], df["V_Y"][1:], label='Yd')
    vel.plot( df["TIME"][1:], df["V_Z"][1:], label='Zd')
    vel.set_xlabel('Time (s)')
    vel.set_ylabel('Velocity (m/s)')
    vel.legend()
    fig2.show()

    fig3 = plt.figure()
    eul = fig3.add_subplot(111)
    eul.plot( df["TIME"][1:], df["YAW"][1:]*180/np.pi, label='Yaw')
    eul.plot( df["TIME"][1:], df["PITCH"][1:]*180/np.pi, label='Pitch')
    eul.plot( df["TIME"][1:], df["ROLL"][1:]*180/np.pi, label='Roll')
    eul.set_xlabel('Time (s)')
    eul.set_ylabel('Euler Angle (deg)')
    eul.legend()
    fig3.show()

    fig4 = plt.figure()
    edot = fig4.add_subplot(111)
    edot.plot( df["TIME"][1:], df["YAW_D"][1:]*180/np.pi, label='Yaw_d')
    edot.plot( df["TIME"][1:], df["PITCH_D"][1:]*180/np.pi, label='Pitch_d')
    edot.plot( df["TIME"][1:], df["ROLL_D"][1:]*180/np.pi, label='Roll_d')
    edot.set_xlabel('Time (s)')
    edot.set_ylabel('Angular Velocity (deg/s)')
    edot.legend()
    fig4.show()

    #---------------------------------------------
    fig5 = plt.figure()
    acc = fig5.add_subplot(211)
    acc.plot( df["TIME"][1:], df["A_XM"][1:], label='X')
    acc.plot( df["TIME"][1:], df["A_YM"][1:], label='Y')
    acc.plot( df["TIME"][1:], df["A_ZM"][1:], label='Z')
    acc.plot( df["TIME"][1:], df["G_X"][1:], label="GravX")
    acc.plot( df["TIME"][1:], df["G_Y"][1:], label="GravY")
    acc.plot( df["TIME"][1:], df["G_Z"][1:], label="GravZ")
    acc.set_xlabel('Time (s)')
    acc.set_ylabel('Measured Acceleration (m/s^2)')
    acc.legend()

    gyr = fig5.add_subplot(212)
    gyr.plot( df["TIME"][1:],  df["Y_DM"][1:], label='Yaw_d')
    gyr.plot( df["TIME"][1:],  df["P_DM"][1:], label='Pitch_d')
    gyr.plot( df["TIME"][1:],  df["R_DM"][1:], label='Roll_d')
    gyr.plot( df["TIME"][1:],  df["GYRO_BX"][1:], label='Bias_Yaw')
    gyr.plot( df["TIME"][1:],  df["GYRO_BY"][1:], label='Bias_Pitch')
    gyr.plot( df["TIME"][1:],  df["GYRO_BZ"][1:], label='Bias_Roll')
    gyr.set_xlabel('Time (s)')
    gyr.set_ylabel('Measured Gyro (rad/s)')
    gyr.legend()
    fig5.show()

    #---------------------------------------------
    fig6 = plt.figure()
    za = fig6.add_subplot(211)
    za.plot( df["TIME"][1:], df["Z_AX"][1:], label='X')
    za.plot( df["TIME"][1:], df["Z_AY"][1:], label='Y')
    za.plot( df["TIME"][1:], df["Z_AZ"][1:], label='Z')
    za.set_xlabel('Time (s)')
    za.set_ylabel('Z Acceleration (m/s^2)')
    za.legend()

    zg = fig6.add_subplot(212)
    zg.plot( df["TIME"][1:], df["Z_GX"][1:], label='Yaw')
    zg.plot( df["TIME"][1:], df["Z_GY"][1:], label='Pitch')
    zg.plot( df["TIME"][1:], df["Z_GZ"][1:], label='Roll')
    zg.set_xlabel('Time (s)')
    zg.set_ylabel('Z Gyro (rad/s)')
    zg.legend()
    fig6.show()
    
    #---------------------------------------------
    dt = []
    for indx, t in enumerate( df['TIME'][1:]):
        dt.append( df['TIME'][indx+1] - df['TIME'][indx])
    
    fig7 = plt.figure()
    kt = fig7.add_subplot(111)
    kt.plot( df['TIME'][1:], dt )
    kt.set_xlabel('Time (s)')
    kt.set_ylabel('Time Step (s)')
    fig7.show()

if __name__ == "__main__":

    pwd = os.getcwd()

    data_dir = "logged_data"
    hrd_dir = "real_hardware"
    data_file = "kalman_data.csv"

    # Read in csv file
    df = pd.read_csv( os.path.join( pwd, data_dir, hrd_dir, data_file ) )
    # df = pd.read_csv( os.path.join( pwd, data_dir, data_file ) )

    plot_kalman_data( df )

    print("done plotting")

