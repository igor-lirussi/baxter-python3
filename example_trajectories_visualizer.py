"""
Description: Visualize in 3D the trajectories recorded in csv. 
             During time (COLUMN 1st) is shown the position of the hand (COLUMN 4th to 10th). Included the gripper open/closed (COLUMN 11th).
             If there is joint data, (COLUMNS 13th to 19th), it also visualizes that in 7 graphs.
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse 
import os

parser = argparse.ArgumentParser()
parser.add_argument('-f', '--filepath', type=str, default='./trajectories/', help='the folder (default ./trajectories/) in which look for trajectory files, a single file can also be passed')
parser.add_argument('-cs', '--cartesianspace', action='store_true', help='Shows 7 graphs for each dimension of cartesian space in time (3D position + 4D quaternion). Overrides joint space graphs if present.')
args = parser.parse_args()

path_provided = args.filepath

files=[]
if os.path.isfile(path_provided) and path_provided.endswith('.csv'):
    files.append(path_provided)
elif os.path.isdir(path_provided):
    for filename in os.listdir(path_provided):
        if filename.endswith('.csv'):
            files.append(os.path.join(path_provided, filename))
else:
    print("Invalid path provided. It is neither a file.csv nor a directory.")

arrays = []
for filename in files:
    if filename.endswith('.csv'):
        print("Loaded: "+filename)
        # Use numpy to load the CSV file into an array
        data = np.genfromtxt(filename, delimiter=',', skip_header=1)
        arrays.append(data)

        # Extract the columns
        px_column = data[:, 3]
        py_column = data[:, 4]
        pz_column = data[:, 5]
        qx_column = data[:, 6]
        qy_column = data[:, 7]
        qz_column = data[:, 8]
        qw_column = data[:, 9]
        gripper_position_column = data[:, 10]

        fig = plt.figure(figsize=(9,7), num=os.path.basename(filename))

        span=4 #default the main plot covers all space

        if args.cartesianspace:
            span=3
            ax1 = plt.subplot2grid((4, 4), (0, 3))
            ax1.plot(data[:,0],px_column)
            ax1.text(0.5, 0.5, "p_x",transform=ax1.transAxes, ha="center", va="center", color="darkgrey")

            ax2 = plt.subplot2grid((4, 4), (1, 3))
            ax2.plot(data[:,0],py_column)
            ax2.text(0.5, 0.5, "p_y",transform=ax2.transAxes, ha="center", va="center", color="darkgrey")
            
            ax3 = plt.subplot2grid((4, 4), (2, 3))
            ax3.plot(data[:,0],pz_column)
            ax3.text(0.5, 0.5, "p_z",transform=ax3.transAxes, ha="center", va="center", color="darkgrey")
            
            ax4 = plt.subplot2grid((4, 4), (3, 3))
            ax4.plot(data[:,0],qx_column)
            ax4.text(0.5, 0.5, "qx",transform=ax4.transAxes, ha="center", va="center", color="darkgrey")
            
            ax5 = plt.subplot2grid((4, 4), (3, 2))
            ax5.plot(data[:,0],qy_column)
            ax5.text(0.5, 0.5, "qy",transform=ax5.transAxes, ha="center", va="center", color="darkgrey")
            
            ax6 = plt.subplot2grid((4, 4), (3, 1))
            ax6.plot(data[:,0],qz_column)
            ax6.text(0.5, 0.5, "qz",transform=ax6.transAxes, ha="center", va="center", color="darkgrey")
            
            ax7 = plt.subplot2grid((4, 4), (3, 0))
            ax7.plot(data[:,0],qw_column)
            ax7.text(0.5, 0.5, "qw",transform=ax7.transAxes, ha="center", va="center", color="darkgrey")
        elif len(data[0])>=13:
            span=3

            ax7 = plt.subplot2grid((4, 4), (3, 0))
            ax7.plot(data[:,0],data[:,12])
            ax7.text(0.5, 0.5, "s0",transform=ax7.transAxes, ha="center", va="center", color="darkgrey")

            if len(data[0])>=14:
                ax6 = plt.subplot2grid((4, 4), (3, 1))
                ax6.plot(data[:,0],data[:,13])
                ax6.text(0.5, 0.5, "s1",transform=ax6.transAxes, ha="center", va="center", color="darkgrey")

            if len(data[0])>=15:
                ax5 = plt.subplot2grid((4, 4), (3, 2))
                ax5.plot(data[:,0],data[:,14])
                ax5.text(0.5, 0.5, "e0",transform=ax5.transAxes, ha="center", va="center", color="darkgrey")

            if len(data[0])>=16:
                ax4 = plt.subplot2grid((4, 4), (3, 3))
                ax4.plot(data[:,0],data[:,15])
                ax4.text(0.5, 0.5, "e1",transform=ax4.transAxes, ha="center", va="center", color="darkgrey")

            if len(data[0])>=17:
                ax3 = plt.subplot2grid((4, 4), (2, 3))
                ax3.plot(data[:,0],data[:,16])
                ax3.text(0.5, 0.5, "w0",transform=ax3.transAxes, ha="center", va="center", color="darkgrey")

            if len(data[0])>=18:
                ax2 = plt.subplot2grid((4, 4), (1, 3))
                ax2.plot(data[:,0],data[:,17])
                ax2.text(0.5, 0.5, "w1",transform=ax2.transAxes, ha="center", va="center", color="darkgrey")

            if len(data[0])>=19:
                ax1 = plt.subplot2grid((4, 4), (0, 3))
                ax1.plot(data[:,0],data[:,18])
                ax1.text(0.5, 0.5, "w2",transform=ax1.transAxes, ha="center", va="center", color="darkgrey")

        #CREATE THE MAIN PLOT
        ax = plt.subplot2grid((4, 4), (0, 0), colspan=span, rowspan=span, projection='3d')
        
        # X goes from 0 to 1 (ahead), Y from -1 (right robot) to 1 (left robot), Z from 0 (table) a 1 (head)
        ax.set_xlim(0, 1)
        ax.set_ylim(-1, 1)
        ax.set_zlim(0, 1)
        # Set non-cubical aspect ratio of the graph
        ax.set_box_aspect([1,2,1])
        
        img_path='./img/baxter-tilted.png'
        if os.path.isfile(img_path):
            # Add an image to the plot
            img = plt.imread(img_path)  
            ax.imshow(img, extent=[-0.07, 0.005, -0.04, 0.07], aspect='auto',  alpha=0.5)  

        scatter = ax.scatter(px_column, py_column, pz_column, c=gripper_position_column, cmap='viridis')

        # Add a colorbar
        cbar = plt.colorbar(scatter)
        cbar.set_label('Gripper Aperture')

        # Set labels for each axis
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        # Show the plot
        plt.tight_layout()
print("You can resize the windows")
print(" [ Ctrl ]+[ C ] here to close all windows")
plt.show()