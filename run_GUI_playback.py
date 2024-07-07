import os 
import genome
import sys
import creature
import pybullet as p
import pybullet_data
import time 
import random
import numpy as np

# ask in console for number
generationNumber = input("Enter the generation number you want to test: ")
subfolder = "csv"
csv_file = 'elite_' + generationNumber + '.csv'
filename = os.path.join(subfolder, csv_file)
print("Loading", filename)
assert os.path.exists(filename), "Tried to load " + filename + " but it does not exists"

# Initialize the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath('shapes/')
p.resetSimulation()
p.setPhysicsEngineParameter(enableFileCaching=0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0, 0, -10)

# Set the camera and debug text
p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

# Create the environment
arena_size = 20
wall_height = 5
wall_thickness = 0.5
floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness])
floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1])
floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])
wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2])
wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size/2, wall_height/2])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size/2, wall_height/2])
wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2])
wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness/2, arena_size/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size/2, 0, wall_height/2])
p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size/2, 0, wall_height/2])
mountain_position = (0, 0, -1)
mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

# generate a random creature
dna = genome.Genome.from_csv(filename)
cr = creature.Creature(gene_count=1)
cr.update_dna(dna)
with open('test.urdf', 'w') as f:
    f.write(cr.to_xml())
rob1 = p.loadURDF('test.urdf')
# air drop it        
p.resetBasePositionAndOrientation(rob1, [5, 5, 2.5], [0, 0, 0, 1])
start_pos, orn = p.getBasePositionAndOrientation(rob1)

# iterate 
elapsed_time = 0
wait_time = 1.0/240 # seconds
total_time = 30 # seconds
step = 0
while True:
    p.stepSimulation()
    step += 1
    if step % 24 == 0:
        motors = cr.get_motors()
        for jid in range(p.getNumJoints(rob1)):
            m = motors[jid]
            p.setJointMotorControl2(rob1, jid, 
                    controlMode=p.VELOCITY_CONTROL, 
                    targetVelocity=motors[jid].get_output(),
                    force = 5)
    pos, _ = p.getBasePositionAndOrientation(rob1)
    cr.update_position(pos)
        
    time.sleep(wait_time)
    elapsed_time += wait_time
    if elapsed_time > total_time:
        break

print("TOTAL DISTANCE MOVED:", cr.get_distance_travelled())