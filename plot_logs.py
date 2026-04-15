import re
import matplotlib.pyplot as plt
import argparse

def plot_robot_logs(log_file):
    # Data storage
    z_offsets = []
    err_x, adj_x = [], []
    err_y, adj_y = [], []

    # Regex to match: [PID Log] Z-Offset: 0.200 | ErrX: 0.0123 (Adj: 0.0098) | ErrY: -0.0045 (Adj: -0.0036)
    pattern = re.compile(
        r"Z-Offset:\s+([-?\d.]+)\s+\|\s+ErrX:\s+([-?\d.]+)\s+\(Adj:\s+([-?\d.]+)\)\s+\|\s+ErrY:\s+([-?\d.]+)\s+\(Adj:\s+([-?\d.]+)\)"
    )

    with open(log_file, 'r') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                z_offsets.append(float(match.group(1)))
                err_x.append(float(match.group(2)))
                adj_x.append(float(match.group(3)))
                err_y.append(float(match.group(4)))
                adj_y.append(float(match.group(5)))

    # Create the Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # Subplot 1: Trajectory Errors
    ax1.plot(z_offsets, err_x, label='Error X', color='red', linestyle='--')
    ax1.plot(z_offsets, err_y, label='Error Y', color='blue', linestyle='--')
    ax1.set_ylabel('Error (meters)')
    ax1.set_title('Trajectory Error vs. Z-Offset')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Subplot 2: PID Adjustments (Control Outputs)
    ax2.plot(z_offsets, adj_x, label='Adj X (Output)', color='red')
    ax2.plot(z_offsets, adj_y, label='Adj Y (Output)', color='blue')
    ax2.set_xlabel('Z-Offset (Insertion Progress)')
    ax2.set_ylabel('PID Adjustment')
    ax2.set_title('Controller Output vs. Z-Offset')
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    # Invert X-axis because Z-offset goes from 0.2 down to 0.0
    plt.gca().invert_xaxis()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize PID error logs.')
    parser.add_argument('file', help='Path to the log file')
    args = parser.parse_args()
    plot_robot_logs(args.file)