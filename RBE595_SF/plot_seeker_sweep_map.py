import os
import pandas as pd

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
plt.style.use('seaborn-whitegrid')

def plot_seeker_sweep_map( df ):
    
    print("Plotting seeker...")
    fig1 = plt.figure()
    ax = fig1.add_subplot( 111, projection = '3d')
    ax.scatter3D( df["P_BOD_X"][1:], df["P_BOD_Y"][1:], df["P_BOD_Z"][1:], marker='.', label='Points')
    ax.scatter3D( df["SKR_BOD_X"][1:], df["SKR_BOD_Y"][1:], df["SKR_BOD_Z"][1:], 'r', marker='.', label='Seeker')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    fig1.suptitle('Map, body frame')
    fig1.tight_layout()
    fig1.show()

    fig2 = plt.figure()
    ax = fig2.add_subplot( 111, projection = '3d')
    ax.scatter3D( df["P_INERT_X"][1:], df["P_INERT_Y"][1:], df["P_INERT_Z"][1:], marker='.', label='Points')
    ax.scatter3D( df["SKR_INERT_X"][1:], df["SKR_INERT_Y"][1:], df["SKR_INERT_Z"][1:], 'r', marker='.', label='Seeker')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    fig2.suptitle('Map, inertial frame')
    fig2.tight_layout()
    fig2.show()
    
    
    
if __name__ == "__main__":
    
    pwd = os.getcwd()
    
    data_dir = "logged_data"
    # hrd_dir = "real_hardware"
    data_file = "seeker_data.csv"
    
    # Read in csv file
    df = pd.read_csv( os.path.join( pwd, data_dir, data_file ) )
    # df = pd.read_csv( os.path.join( pwd, data_dir, hrd_dir, data_file ) )
    
    plot_seeker_sweep_map( df )
    
    print("done plotting")