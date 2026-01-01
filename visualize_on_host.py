import socket
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def main():
    # Configuration
    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)

    print(f"Listening for data on {UDP_IP}:{UDP_PORT}...")

    # Set up Matplotlib
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-5, 25)
    ax.set_ylim(-5, 25)
    ax.set_title("Swarm Flight Visualization (Host)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)

    drone_plots = {}
    goal_plot, = ax.plot([], [], 'rx', markersize=10, label='Goal')

    colors = {'1': 'blue', '2': 'green', '3': 'orange', '4': 'purple'}
    data_store = {"drones": {}, "goal": None}

    def init():
        goal_plot.set_data([], [])
        return goal_plot,

    def update(frame):
        # Try to receive data
        try:
            data, addr = sock.recvfrom(1024)
            new_data = json.loads(data.decode())
            data_store.update(new_data)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"Error receiving: {e}")

        # Update Drones
        for drone_id, pos in data_store["drones"].items():
            if drone_id not in drone_plots:
                drone_plots[drone_id], = ax.plot([], [], 'o', color=colors.get(drone_id, 'black'), label=f'Drone {drone_id}')
                ax.legend()
            drone_plots[drone_id].set_data([pos[0]], [pos[1]])

        # Update Goal
        if data_store["goal"]:
            goal_plot.set_data([data_store["goal"][0]], [data_store["goal"][1]])

        return list(drone_plots.values()) + [goal_plot]

    ani = FuncAnimation(fig, update, frames=None, init_func=init, blit=True, interval=50)

    plt.show()
    sock.close()

if __name__ == '__main__':
    main()
