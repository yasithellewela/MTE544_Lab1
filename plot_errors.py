import matplotlib.pyplot as plt
from utilities import FileReader




def plot_pose(filename , u=0):
    
    u = u+1
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))


    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[0].set_title("PID Sigmoid Trajectory")
    axes[0].set_xlabel("X")
    axes[0].set_ylabel("Y")
    axes[0].grid()

    
    axes[1].set_title("Variables Against Time")
    for i in range(0, len(headers) - u):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    axes[1].legend()
    axes[1].set_xlabel("Time(ns)")
    axes[1].set_ylabel("X, Y Position / Theta")
    axes[1].grid()

    plt.show()


def plot_errors(iteration):
    # File naming convention for linear and angular errors
    linear_file = f"linear copy {iteration}.csv"
    angular_file = f"angular copy {iteration}.csv"

    # Read data from both files
    linear_headers, linear_values = FileReader(linear_file).read_file()
    angular_headers, angular_values = FileReader(angular_file).read_file()

    # Extract timestamps and normalize time
    first_linear_stamp = linear_values[0][-1]
    first_angular_stamp = angular_values[0][-1]
    
    linear_time = [val[-1] - first_linear_stamp for val in linear_values]
    angular_time = [val[-1] - first_angular_stamp for val in angular_values]
    
    # Extract error values
    linear_e = [lin[0] for lin in linear_values]
    linear_e_dot = [lin[1] for lin in linear_values]
    angular_e = [ang[0] for ang in angular_values]
    angular_e_dot = [ang[1] for ang in angular_values]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Plot e vs e_dot for both linear and angular errors
    axes[0].plot(linear_e, linear_e_dot, label="Linear Error")
    axes[0].plot(angular_e, angular_e_dot, label="Angular Error", linestyle='--')
    axes[0].set_xlabel("e")
    axes[0].set_ylabel("e_dot")
    axes[0].set_title("e vs e_dot")
    axes[0].legend()
    axes[0].grid()

    # Plot e and e_dot vs time for both linear and angular errors
    axes[1].plot(linear_time, linear_e, label="Linear e")
    axes[1].plot(linear_time, linear_e_dot, label="Linear e_dot")
    axes[1].plot(angular_time, angular_e, label="Angular e", linestyle='--')
    axes[1].plot(angular_time, angular_e_dot, label="Angular e_dot", linestyle='--')
    axes[1].set_xlabel("Time")
    axes[0].set_ylabel("errors")
    axes[1].set_title("Error Variables Against Time")
    axes[1].legend()
    axes[1].grid()

    plt.show()


import argparse

if __name__=="__main__":

    iteration = 6
    plot_errors(iteration)
    plot_pose(f"robot_pose copy {iteration}.csv")

    # parser = argparse.ArgumentParser(description='Process some files.')
    # parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    # parser.add_argument('--u', type=int, required=False, default=0, help='List of files to process')
    
    # args = parser.parse_args()
    
    # print("plotting the files", args.files)

    # filenames=args.files
    # u=args.u
    # for filename in filenames:
    #     plot_pose(filename, u)
  




