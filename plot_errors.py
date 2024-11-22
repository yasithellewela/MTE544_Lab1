import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,6))

    
    # Predicted vals from KF 
    predicted_x = [lin[len(headers) - 9] for lin in values]
    predicted_y = [lin[len(headers) - 8] for lin in values]

    # Actual vals from odom
    actual_x = [lin[len(headers) - 7] for lin in values] 
    actual_y = [lin[len(headers) - 6] for lin in values] 

    # First plot: Overlayed state space
    axes[0].plot(predicted_x, predicted_y, label="Predicted Path", color="blue")
    axes[0].plot(actual_x, actual_y, label="Actual Path", color="red")
    axes[0].set_title("State Space: Predicted vs Actual Path")
    axes[0].grid()
    axes[0].legend()
    

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

    plt.show()
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


