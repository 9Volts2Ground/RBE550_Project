
import numpy as np
import os
import pandas as pd

# Plotting Libraries
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('seaborn-whitegrid')

# Custom libraries
import hardware

def plot_foot_trajectory(
        df,
        motorMin,
        motorMax,
        motorCenter,
        motorOrientation
        ):

    print("plot_foot_trajectory...")

    # Sort df values
    foot_pos = np.zeros( ( 3, 6, df['TIME'].count() ) )
    foot_pos[0,0,:] = df['FOOT_1_X']
    foot_pos[1,0,:] = df['FOOT_1_Y']
    foot_pos[2,0,:] = df['FOOT_1_Z']
    foot_pos[0,1,:] = df['FOOT_2_X']
    foot_pos[1,1,:] = df['FOOT_2_Y']
    foot_pos[2,1,:] = df['FOOT_2_Z']
    foot_pos[0,2,:] = df['FOOT_3_X']
    foot_pos[1,2,:] = df['FOOT_3_Y']
    foot_pos[2,2,:] = df['FOOT_3_Z']
    foot_pos[0,3,:] = df['FOOT_4_X']
    foot_pos[1,3,:] = df['FOOT_4_Y']
    foot_pos[2,3,:] = df['FOOT_4_Z']
    foot_pos[0,4,:] = df['FOOT_5_X']
    foot_pos[1,4,:] = df['FOOT_5_Y']
    foot_pos[2,4,:] = df['FOOT_5_Z']
    foot_pos[0,5,:] = df['FOOT_6_X']
    foot_pos[1,5,:] = df['FOOT_6_Y']
    foot_pos[2,5,:] = df['FOOT_6_Z']

    foot_position_inertial = np.zeros( ( 3, 6, df['TIME'].count() ) )
    foot_position_inertial[0,0,:] = df['FOOT_I_1_X']
    foot_position_inertial[1,0,:] = df['FOOT_I_1_Y']
    foot_position_inertial[2,0,:] = df['FOOT_I_1_Z']
    foot_position_inertial[0,1,:] = df['FOOT_I_2_X']
    foot_position_inertial[1,1,:] = df['FOOT_I_2_Y']
    foot_position_inertial[2,1,:] = df['FOOT_I_2_Z']
    foot_position_inertial[0,2,:] = df['FOOT_I_3_X']
    foot_position_inertial[1,2,:] = df['FOOT_I_3_Y']
    foot_position_inertial[2,2,:] = df['FOOT_I_3_Z']
    foot_position_inertial[0,3,:] = df['FOOT_I_4_X']
    foot_position_inertial[1,3,:] = df['FOOT_I_4_Y']
    foot_position_inertial[2,3,:] = df['FOOT_I_4_Z']
    foot_position_inertial[0,4,:] = df['FOOT_I_5_X']
    foot_position_inertial[1,4,:] = df['FOOT_I_5_Y']
    foot_position_inertial[2,4,:] = df['FOOT_I_5_Z']
    foot_position_inertial[0,5,:] = df['FOOT_I_6_X']
    foot_position_inertial[1,5,:] = df['FOOT_I_6_Y']
    foot_position_inertial[2,5,:] = df['FOOT_I_6_Z']

    joint_ang = np.zeros( ( 3, 6, df['TIME'].count() ) )
    joint_ang[0,0,:] = df['JOINT_1_X']
    joint_ang[1,0,:] = df['JOINT_1_Y']
    joint_ang[2,0,:] = df['JOINT_1_Z']
    joint_ang[0,1,:] = df['JOINT_2_X']
    joint_ang[1,1,:] = df['JOINT_2_Y']
    joint_ang[2,1,:] = df['JOINT_2_Z']
    joint_ang[0,2,:] = df['JOINT_3_X']
    joint_ang[1,2,:] = df['JOINT_3_Y']
    joint_ang[2,2,:] = df['JOINT_3_Z']
    joint_ang[0,3,:] = df['JOINT_4_X']
    joint_ang[1,3,:] = df['JOINT_4_Y']
    joint_ang[2,3,:] = df['JOINT_4_Z']
    joint_ang[0,4,:] = df['JOINT_5_X']
    joint_ang[1,4,:] = df['JOINT_5_Y']
    joint_ang[2,4,:] = df['JOINT_5_Z']
    joint_ang[0,5,:] = df['JOINT_6_X']
    joint_ang[1,5,:] = df['JOINT_6_Y']
    joint_ang[2,5,:] = df['JOINT_6_Z']

    pose = np.zeros( ( df['TIME'].count(), 3 ) )
    pose[:,0] = df['BODY_X']
    pose[:,1] = df['BODY_Y']
    pose[:,2] = df['BODY_Z']

    #---------------------------------------------------------------------
    # Plot 3d foot trajectories
    fig1 = plt.figure(1)
    ax = fig1.add_subplot(111, projection = '3d')
    for leg in range(6):
        ax.plot3D( foot_pos[0,leg,1:], foot_pos[1,leg,1:], foot_pos[2,leg,1:], label=str('Leg'+str(leg+1)), marker='.' )
        # ax.plot3D( foot_pos[0,leg,1], foot_pos[1,leg,1], foot_pos[2,leg,1], label=str('Leg'+str(leg+1)), marker='*' )
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    fig1.suptitle('Foot positions, body frame')
    fig1.show()

    #---------------------------------------------------------------------
    # Plot joint angles
    fig2, (shoulder, knee, ankle) = plt.subplots(nrows = 3, ncols = 1)
    for leg in range(6):
        shoulder.plot( df['TIME'], joint_ang[0,leg,:], label=str('Leg'+str(leg+1)), marker='.' )
        knee.plot( df['TIME'], joint_ang[1,leg,:], label=str('Leg'+str(leg+1)), marker='.' )
        ankle.plot( df['TIME'], joint_ang[2,leg,:], label=str('Leg'+str(leg+1)), marker='.' )
    shoulder.set_xlabel('Time (s)')
    shoulder.set_ylabel('Shoulder Angles (rad)')
    shoulder.legend()

    knee.set_xlabel('Time (s)')
    knee.set_ylabel('Knee Angles (rad)')
    knee.legend()

    ankle.set_xlabel('Time (s)')
    ankle.set_ylabel('Ankle Angles (rad)')
    knee.legend()
    fig2.suptitle('Joint angles')
    fig2.show()

    #---------------------------------------------------------------------
    # Plot joint angles with joint limits
    fig3, sj = plt.subplots(3, 2)
    fig4, kj = plt.subplots(3,2)
    fig5, aj = plt.subplots(3,2)
    lg = 0
    for row in range(3):
        for col in range(2):

            sj[row,col].plot(df['TIME'], joint_ang[0,lg,:], label=str('Leg'+str(lg+1)), marker='.')
            kj[row,col].plot(df['TIME'], joint_ang[1,lg,:], label=str('Leg'+str(lg+1)), marker='.')
            aj[row,col].plot(df['TIME'], joint_ang[2,lg,:], label=str('Leg'+str(lg+1)), marker='.')

            sj[row,col].legend()
            kj[row,col].legend()
            aj[row,col].legend()

            sj[row,col].hlines( (motorMin[0,lg]-motorCenter[0,lg])*np.pi/180, df['TIME'].iloc[0], df['TIME'].iloc[-1])
            sj[row,col].hlines( (motorMax[0,lg]-motorCenter[0,lg])*np.pi/180, df['TIME'].iloc[0], df['TIME'].iloc[-1])
            sj[row,col].set_xlabel('Time (s)')
            sj[row,col].set_ylabel('Shoulder Joint Angles (rad)')

            kj[row,col].hlines( (motorMin[1,lg]-motorCenter[1,lg])*np.pi/180, df['TIME'].iloc[0], df['TIME'].iloc[-1])
            kj[row,col].hlines( (motorMax[1,lg]-motorCenter[1,lg])*np.pi/180, df['TIME'].iloc[0], df['TIME'].iloc[-1])
            kj[row,col].set_xlabel('Time (s)')
            kj[row,col].set_ylabel('Knee Joint Angles (rad)')

            aj[row,col].hlines( (motorMin[2,lg]-motorCenter[2,lg])*np.pi/180*motorOrientation[2,lg], df['TIME'].iloc[0], df['TIME'].iloc[-1])
            aj[row,col].hlines( (motorMax[2,lg]-motorCenter[2,lg])*np.pi/180*motorOrientation[2,lg], df['TIME'].iloc[0], df['TIME'].iloc[-1])
            aj[row,col].set_xlabel('Time (s)')
            aj[row,col].set_ylabel('Ankle Joint Angles (rad)')

            lg += 1

    fig3.show()
    fig4.show()
    fig5.show()

    #---------------------------------------------------------------------
    # Plot foot position in inertial frame
    fig6 = plt.figure()
    ax6 = fig6.add_subplot(111, projection = '3d')
    ax6.plot3D( pose[:,0], pose[:,1], pose[:,2], marker='.', label='body')
    for leg in range(6):
        ax6.plot3D( foot_position_inertial[0,leg,:], foot_position_inertial[1,leg,:], foot_position_inertial[2,leg,:], label=str('Leg'+str(leg+1)), marker='.' )

    ax6.set_xlabel('X (m)')
    ax6.set_ylabel('Y (m)')
    ax6.set_zlabel('Z (m)')
    ax6.legend()
    fig6.suptitle('Body/Foot positions, Inertial')
    fig6.show()

    #---------------------------------------------------------------------
    # Plot body position over time
    fig7 = plt.figure()
    ax7 = fig7.add_subplot(111)
    ax7.plot( df['TIME'], pose[:,0], marker='.', label='X')
    ax7.plot( df['TIME'], pose[:,1], marker='.', label='Y')
    ax7.plot( df['TIME'], pose[:,2], marker='.', label='Z')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Position (m)')
    ax7.legend()
    fig7.suptitle('Body Position, Inertial Frame')
    fig7.show()

#==============================================================================
if __name__ == "__main__":

    pwd = os.getcwd()

    data_dir = "logged_data"
    hrd_dir = "real_hardware"
    data_file = "foot_trajectory_data.csv"

    # Read in csv file
    # df = pd.read_csv( os.path.join( pwd, data_dir, data_file ) )
    df = pd.read_csv( os.path.join( pwd, data_dir, hrd_dir, data_file ) )

    hrd = hardware.hardware()

    plot_foot_trajectory(
        df,
        hrd.motorMin,
        hrd.motorMax,
        hrd.motorCenter,
        hrd.motorOrientation
        )

    print("done plotting")

