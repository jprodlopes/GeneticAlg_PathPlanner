import random
from genetic_base import Individual
import numpy as np
from config import B_MIN, B_MAX

# Define the problem-specific parameters
POPULATION_SIZE = 100  #Must be even
GENERATION_COUNT = 0
SELECTION = "Roulette"
CROSSOVER_RATE = 0.8
MUTATION_RATE = 0.2
MUTATION_RATE_TANGENCY = 0.3
MUTATION_RATE_WINGSPAN_TURNS = 0.1
MUTATION_RATE_WINGSPAN_STRAIGHT = 0.5

class Population:
    def __init__(self, x_init, x_goal, obs, params = None):
        if params:
            global POPULATION_SIZE, GENERATION_COUNT, SELECTION
            POPULATION_SIZE, GENERATION_COUNT, SELECTION = params[0], params[1], params[2]
        self.x_init = x_init
        self.x_goal = x_goal
        self.obs = obs
        self.n_obs =len(obs)
        self.perf, self.ind_selected = [], []
        self.perf_iteration = 9999

        # Initialization 
        print("Initializing population")
        self.pop  = [Individual(x_init, x_goal, obs) for i in range(POPULATION_SIZE)]
        self.pop[POPULATION_SIZE-1] = Individual(x_init, x_goal, obs, ['RT'] +['NT']*(self.n_obs-2) + ['LT'])
        self.popInit = self.pop
        self.best_indv = self.pop[0]

    def selection(self):
        if SELECTION == "Roulette":
            self.roulette_wheel_selection()
        if SELECTION == "Tournament":
            self.tournament_selection()
        if SELECTION == "BestHalf":
            self.best_half_selection()
        if SELECTION == "Rank":
            self.rank_selection()

        # Always keep the best individual from the previous generation
        self.ind_selected[-1] = self.best_indv


    def best_half_selection(self):
        pop = sorted(self.pop, key=lambda x: x.score)
        self.ind_selected = pop[:POPULATION_SIZE//2]

    def tournament_selection(self, tournament_size=2):
        population = self.pop
        selected_individuals = []
        population_size = len(population)
        scores = [idv.score for idv in population]

        for _ in range(population_size):
            # Randomly select individuals for the tournament
            tournament_indices = random.sample(range(population_size), tournament_size)
            tournament_candidates = [population[i] for i in tournament_indices]
            tournament_fitness = [scores[i] for i in tournament_indices]

            # Choose the individual with the highest fitness from the tournament
            winner_index = tournament_fitness.index(min(tournament_fitness))
            selected_individuals.append(tournament_candidates[winner_index])

        self.ind_selected = selected_individuals
    
    
    def roulette_wheel_selection(self):  
        population = self.pop
        fitness_scores = np.sum([10000/ind.score for ind in population])

        probabilities = [(10000/ind.score) / fitness_scores for ind in population]
        selected_individuals = []
        for _ in range(len(population)):
            # Spin the roulette wheel
            spin = random.uniform(0, 1)
            cumulative_probability = 0

            # Find the selected individual based on the roulette wheel
            for i, prob in enumerate(probabilities):
                cumulative_probability += prob
                if spin <= cumulative_probability:
                    selected_individuals.append(population[i])
                    break
        self.ind_selected =  selected_individuals

    def rank_selection(self, alpha=0.05):
        # Sort by score (lower is better)
        sorted_pop = sorted(self.pop, key=lambda x: x.score)
        n = len(sorted_pop)

        # rank 0 is best
        r = np.arange(n)

        # exponential decay: best gets weight 1, then exp(-alpha), exp(-2alpha), ...
        weights = np.exp(-alpha * r)
        probs = weights / weights.sum()
        idxs = np.random.choice(n, size=n, p=probs, replace=True)
        self.ind_selected = [sorted_pop[i] for i in idxs]


    def crossover(self):
        # Remove bad individuals
        self.pop[:int(len(self.ind_selected))] = []

        options = self.ind_selected
        for i in range(int(len(self.ind_selected)/2)):
                parent1 = random.choice(options)
                options.remove(parent1)
                parent2 = random.choice(options)
                options.remove(parent2)

                if(random.random() < CROSSOVER_RATE):
                    child1, child2 = self.breed_parents(parent1, parent2)
                    child1 = self.mutate(child1)
                    child2 = self.mutate(child2)
                    self.pop.append(child1)
                    self.pop.append(child2)
                else:
                    self.pop.append(parent1)
                    self.pop.append(parent2)

    def breed_parents(self, parent1, parent2):
        return self.breed_parents3(parent1, parent2 )

    def breed_parents1(self, parent1, parent2):
        p1 = random.randint(0, (self.n_obs))
        p2 = random.randint(p1, (self.n_obs))

        if(random.random() < 0.5):
            #Keep parameters, crossover tangencies
            g1_tan = parent1.tangency[0:p1] + parent2.tangency[p1:p2] + parent1.tangency[p2:]
            g2_tan = parent2.tangency[0:p1] + parent1.tangency[p1:p2] + parent2.tangency[p2:]
            g1_speed = parent1.v
            g2_speed = parent2.v
            g1_wingspan = parent1.wingspan
            g2_wingspan = parent2.wingspan
            g1_r_min = parent1.r_min
            g2_r_min = parent2.r_min
        else:
            #Keep tangencies, crossover vehicle parameters
            g1_tan = parent1.tangency
            g2_tan = parent2.tangency

            mask = np.random.rand(len(parent1.v)) < 0.5
            g1_speed = np.where(mask, parent1.v, parent2.v)
            g2_speed = np.where(mask, parent2.v, parent1.v)
            g1_wingspan = np.where(mask, parent1.wingspan, parent2.wingspan)
            g2_wingspan = np.where(mask, parent2.wingspan, parent1.wingspan)
            g1_r_min = np.where(mask, parent1.r_min, parent2.r_min)
            g2_r_min = np.where(mask, parent2.r_min, parent1.r_min)
        # Create individuals
        child_1 = Individual(self.x_init, self.x_goal, self.obs, g1_tan, [g1_speed, g1_wingspan, g1_r_min]) 
        child_2 = Individual(self.x_init, self.x_goal, self.obs, g2_tan, [g2_speed, g2_wingspan, g2_r_min]) 
        return child_1, child_2

    def breed_parents2(self, parent1, parent2):
        mask = np.random.rand(len(parent1.tangency)) < 0.5
        mask2 = np.repeat(mask,2)
        g1_tan = np.where(mask, parent1.tangency, parent2.tangency)
        g2_tan = np.where(mask, parent2.tangency, parent1.tangency)    
        g1_speed = np.where(mask2, parent1.v, parent2.v)
        g2_speed = np.where(mask2, parent2.v, parent1.v)
        g1_wingspan = np.where(mask2, parent1.wingspan, parent2.wingspan)
        g2_wingspan = np.where(mask2, parent2.wingspan, parent1.wingspan)
        g1_r_min = np.where(mask2, parent1.r_min, parent2.r_min)
        g2_r_min = np.where(mask2, parent2.r_min, parent1.r_min)
        # Create individuals
        child_1 = Individual(self.x_init, self.x_goal, self.obs, g1_tan, [g1_speed, g1_wingspan, g1_r_min]) 
        child_2 = Individual(self.x_init, self.x_goal, self.obs, g2_tan, [g2_speed, g2_wingspan, g2_r_min]) 
        return child_1, child_2
    

    def breed_parents3(self, parent1, parent2):
        #single point crossover for tangency, uniform crossover for vehicle parameters
        p1 = random.randint(0, (self.n_obs))
        g1_tan = parent1.tangency[0:p1] + parent2.tangency[p1:]
        g2_tan = parent2.tangency[0:p1] + parent1.tangency[p1:]
        mask = np.random.rand(len(parent1.v)) < 0.5
        g1_speed = np.where(mask, parent1.v, parent2.v)
        g2_speed = np.where(mask, parent2.v, parent1.v)
        g1_wingspan = np.where(mask, parent1.wingspan, parent2.wingspan)
        g2_wingspan = np.where(mask, parent2.wingspan, parent1.wingspan)
        g1_r_min = np.where(mask, parent1.r_min, parent2.r_min)
        g2_r_min = np.where(mask, parent2.r_min, parent1.r_min)
        # Create individuals
        child_1 = Individual(self.x_init, self.x_goal, self.obs, g1_tan, [g1_speed, g1_wingspan, g1_r_min]) 
        child_2 = Individual(self.x_init, self.x_goal, self.obs, g2_tan, [g2_speed, g2_wingspan, g2_r_min]) 
        return child_1, child_2
    
    def mutate(self, idv):
        if (random.random() < MUTATION_RATE):
            # Mutate tangency
            if(random.random() < MUTATION_RATE_TANGENCY):
                p1 = random.randint(1, self.n_obs-2)
                mode = ["RT", "LT", "NT","NT","NT"]
                idv.tangency[p1] = (random.choice(mode))

            # Mutate wingspan and r_min
            while(random.random() < MUTATION_RATE_WINGSPAN_TURNS):
                p1 = random.randint(1, self.n_obs-2)
                # Mutate wingspan during arccircles
                idv.wingspan[2*p1] = random.uniform(B_MIN, B_MAX)
                idv.randomize_vehiclePars(p1)
            while(random.random() < MUTATION_RATE_WINGSPAN_STRAIGHT):
                p1 = random.randint(1, self.n_obs-2)   
                # Mutate wingspan during straightlines
                idv.wingspan[2*p1+1] = random.uniform(B_MIN, B_MAX)
                idv.randomize_vehiclePars(p1)
        return idv

    def performance(self, iteration):
        total_score = 0
        for i in range(len(self.pop)):
            total_score += self.pop[i].score
        self.perf_iteration = int(total_score)
        self.perf.append([iteration, int(self.perf_iteration)])
    


def run_algo(x_init, x_goal, obs, params):
    # Initialize population
    P = Population(x_init, x_goal, obs, params)
    best_idv, best_score = [], np.inf

    for iteration in range(GENERATION_COUNT):
        P.selection()
        P.crossover()
        P.performance(iteration)
        ranking = sorted(P.pop, key=lambda x: x.score)  
        if ranking[0].score < best_score:
            best_idv = ranking[0]
        #print(  "Iteration " + str(iteration) + " Perf " + 
        #        str(int(P.perf_iteration/1000)) + " Population size " + str(len(P.pop)))       
    return best_idv, P.perf