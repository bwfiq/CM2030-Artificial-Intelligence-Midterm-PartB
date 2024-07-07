import os 
import genome
import creature
import pybullet as p
import time
import numpy as np
import environment
            
# ask in console for number
generationNumber = input("Enter the generation number you want to test: ")
subfolder = "csv"
csv_file = 'elite_' + generationNumber + '.csv'
filename = os.path.join(subfolder, csv_file)
print("Loading", filename)
assert os.path.exists(filename), "Tried to load " + filename + " but it does not exists"

# Initialize the simulation
pid = p.connect(p.GUI)
environment.Environment.init_environment(pid, 20)

# Set the pybullet parameters
p.setPhysicsEngineParameter(enableFileCaching=0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

# generate a random creature
dna = genome.Genome.from_csv(filename)
cr = creature.Creature(gene_count=1)
cr.update_dna(dna)
with open('test.urdf', 'w') as f:
    f.write(cr.to_xml())
rob1 = p.loadURDF('test.urdf')
# air drop it        
p.resetBasePositionAndOrientation(rob1, [-5, -5, 2.5], [0, 0, 0, 1])
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