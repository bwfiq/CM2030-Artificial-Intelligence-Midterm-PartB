import pybullet as p
from multiprocessing import Pool
import environment

class Simulation: 
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id
        self.environment_state_id = None
        self.arena_size = 20        
 
    def run_creature(self, cr, iterations=2400):     
        pid = self.physicsClientId
        if (self.environment_state_id is not None):
            p.restoreState(stateID=self.environment_state_id, physicsClientId=pid)
        else:
            environment.Environment.init_environment(pid, 20)
            self.environment_state_id = p.saveState(physicsClientId=pid)

        # load in the creature
        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        with open(xml_file, 'w') as f:
            f.write(cr.to_xml())
        cid = p.loadURDF(xml_file, physicsClientId=pid)
        p.resetBasePositionAndOrientation(cid, [-5, -5, 2.5], [0, 0, 0, 1], physicsClientId=pid)

        # Run the simulation
        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)
            
            # Move the creature's motors
            if step % 24 == 0:
                for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
                    m = cr.get_motors()[jid]
                    p.setJointMotorControl2(cid, jid, 
                            controlMode=p.VELOCITY_CONTROL, 
                            targetVelocity=m.get_output(), 
                            force = 5, 
                            physicsClientId=self.physicsClientId)
                    
            # Update the creature's position
            pos, _ = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
            # Kill the creature if it goes out of bounds
            if pos[0] > self.arena_size/2 or pos[0] < -self.arena_size/2 or pos[1] > self.arena_size/2 or pos[1] < -self.arena_size/2:
                cr.set_fitness(0)
                break
            else:
                cr.update_position(pos)
        cr.set_fitness(cr.get_distance_travelled())

class ThreadedSim:
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):# the end
                    break
                # work out the sim ind
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
                # it works on a copy of the creatures, so receive them
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                # and now put those creatures back into the main 
                # self.creatures array
                new_creatures.extend(creatures)
        pop.creatures = new_creatures
