import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # Import Poly3DCollection
import serial


def rotate_x(angle):
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])

def rotate_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])

def rotate_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

# Initialize figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

scale_factor = 1  # Adjust the scale factor as needed
# Define cube vertices
cube_vertices = np.array([
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.5, 0.5, -0.5],
    [-0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5],
    [0.5, 0.5, 0.5],
    [-0.5, 0.5, 0.5]
]) * scale_factor  # Scale down the vertices

# Define cube faces
cube_faces = [
    [cube_vertices[0], cube_vertices[1], cube_vertices[2], cube_vertices[3]],  # Front face
    [cube_vertices[4], cube_vertices[5], cube_vertices[6], cube_vertices[7]],  # Back face
    [cube_vertices[0], cube_vertices[1], cube_vertices[5], cube_vertices[4]],  # Bottom face
    [cube_vertices[3], cube_vertices[2], cube_vertices[6], cube_vertices[7]],  # Top face
    [cube_vertices[0], cube_vertices[3], cube_vertices[7], cube_vertices[4]],  # Left face
    [cube_vertices[1], cube_vertices[2], cube_vertices[6], cube_vertices[5]]   # Right face
]

# Define colors for each face with slight transparency
face_colors = ['b', 'g', 'r', 'c', 'm', 'y']
face_alphas = [1] * 6  # Transparency level (adjust as needed)

# Plot initial cube faces with colors and transparency
cube_collections = []
for face, color, alpha in zip(cube_faces, face_colors, face_alphas):
    cube_collections.append(Poly3DCollection([face], color=color, alpha=alpha))  # Use Poly3DCollection
    ax.add_collection3d(cube_collections[-1])


# Set axis limits
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# Connect to Arduino serial port
ser = serial.Serial('COM3', 9600)  # Change 'COM3' to your Arduino's serial port

# Initialize angles
pitch = 0
yaw = 0
roll = 0

# Function to update cube faces
def update_cube_faces():
    for collection, face in zip(cube_collections, cube_faces):
        collection.set_verts([face])

while True:
    # Read data from Arduino
    arduino_data = ser.readline().decode().strip()
    print(arduino_data)
    # Skip empty lines
    if not arduino_data:
        continue
    
    # Split data
    data_parts = arduino_data.split(':')
    if len(data_parts) < 2:
        continue
    
    # Extract parameter and value
    parameter = data_parts[0].strip()
    value = data_parts[1].strip()
    
    # Extract numerical value
    numerical_value = value.split('°')[0].strip()

    # Convert to float
    try:
        numerical_value = float(numerical_value)
    except ValueError:
        continue
    
    # Assign numerical value based on parameter
    if parameter == 'Pitch':
        pitch = numerical_value
    elif parameter == 'Roll':
        roll = numerical_value
    elif parameter == 'Yaw':
        yaw = numerical_value
    
    # Rotation matrices
    R_pitch = rotate_x(np.radians(pitch))
    R_yaw = rotate_y(np.radians(roll))
    R_roll = rotate_z(0)#np.radians(roll))
    
    # Apply rotations
    rotated_cube = np.dot(cube_vertices, R_pitch)
    rotated_cube = np.dot(rotated_cube, R_yaw)
    rotated_cube = np.dot(rotated_cube, R_roll)
    
    # Update cube faces
    for i, face in enumerate(cube_faces):
        rotated_face = np.dot(face, R_pitch)
        rotated_face = np.dot(rotated_face, R_yaw)
        rotated_face = np.dot(rotated_face, R_roll)
        cube_collections[i].set_verts([rotated_face])
    
    # Update plot
    plt.pause(0.01)
