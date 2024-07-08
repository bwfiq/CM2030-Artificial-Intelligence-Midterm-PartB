import pybullet as p
import pybullet_data
import numpy as np
from multiprocessing import Pool

class Simulation: 
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id
        
        # Initialize the environment and save the state so it doesn't have to be built every time
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.physicsClientId)
        p.setAdditionalSearchPath('shapes/', physicsClientId=self.physicsClientId)
        p.resetSimulation(physicsClientId=self.physicsClientId)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=self.physicsClientId)
        p.setGravity(0, 0, -10, physicsClientId=self.physicsClientId)
        
        arena_size = 20
        wall_height = 1
        wall_thickness = 0.5
        floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], physicsClientId=self.physicsClientId)
        floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, arena_size/2, wall_thickness], rgbaColor=[1, 1, 0, 1], physicsClientId=self.physicsClientId)
        floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness], physicsClientId=self.physicsClientId)
        wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], physicsClientId=self.physicsClientId)
        wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size/2, wall_thickness/2, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1], physicsClientId=self.physicsClientId)
        wall_positions = [[0, arena_size/2, wall_height/2], [0, -arena_size/2, wall_height/2], 
                        [arena_size/2, 0, wall_height/2], [-arena_size/2, 0, wall_height/2]]
        for pos in wall_positions:
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=pos, physicsClientId=self.physicsClientId)
        mountain_position = (0, 0, -1)
        mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
        mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1, physicsClientId=self.physicsClientId)
        self.env_id = p.saveState(physicsClientId=self.physicsClientId)

    def run_creature(self, cr, iterations=2400):
        pid = self.physicsClientId
        p.restoreState(self.env_id, physicsClientId=pid) # restore the environment (arena, mountain) without the creature 
        
        # Load in the creature
        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        with open(xml_file, 'w') as f:
            f.write(cr.to_xml())
        cid = p.loadURDF(xml_file, physicsClientId=pid)
        p.resetBasePositionAndOrientation(cid, [5, 5, 2.5], [0, 0, 0, 1], physicsClientId=pid)

        # Run the simulation
        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)
            if step % 240 == 0:
                for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
                    m = cr.get_motors()[jid]
                    p.setJointMotorControl2(cid, jid, 
                            controlMode=p.VELOCITY_CONTROL, 
                            targetVelocity=m.get_output(), 
                            force = 5, 
                            physicsClientId=self.physicsClientId)
            pos, _ = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
            cr.update_position(pos)
        
        # Update fitness
        distance_from_peak = np.linalg.norm(np.array(pos) - np.array([0, 0, 4]))
        cr.set_fitness(1.0 / distance_from_peak)
        p.removeBody(cid, physicsClientId=pid) # clean up for next run

class ThreadedSim:
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):
                    break
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                            self.sims[sim_ind], 
                            pop.creatures[i], 
                            iterations]   
                )
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                new_creatures.extend(creatures)
        pop.creatures = new_creatures