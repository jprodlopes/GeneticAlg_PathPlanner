# Copilot Instructions for Genetic Algorithm Path Planner

## Architecture Overview

This codebase implements a **genetic algorithm-based path planner for morphing wing drones navigating obstacle fields**. The system evolves drone trajectories (via UAV kinematics constraints) to minimize flight time while avoiding circular obstacles.

### Core Data Flow

1. **Map Creation** (`obstacles.py`): Generates 15 non-overlapping circular obstacles with start/goal points on first/last circles
2. **Population Initialization** (`genetic_algo.py`): Creates population of `Individual` paths
3. **Individual Path** (`genetic_base.py`): Each drone path encodes:
   - **Tangency mode per obstacle** (RT/LT/NT): How drone approaches each circle
   - **Vehicle parameters per segment**: Speed (`v`), wingspan (`b`), minimum turn radius (`r_min`)
4. **Path Construction** (`dubins_path.py`): Builds smooth paths via Dubins curves connecting obstacle circles
5. **Fitness Evaluation**: Time + collision penalty (see [genetic_base.py](genetic_base.py#L142))
6. **Evolution**: Selection → Crossover (split genes at random obstacle) → Mutation → Next generation

### Key Architectural Decisions

- **Tangency encoding (RT/LT/NT)** vs. actual geometry: Reduces search space; dubins path generation is deterministic per mode
- **No-turn mode (NT)**: Consecutive "no-turn" obstacles are skipped; forces drone to decide turn direction at next turn obstacle
- **Minimum radius enforcement**: `max(obstacle_radius + wingspan, r_min)` prevents physically impossible turns
- **Fitness formula**: `200 * coeff * collision_points + total_time` where `coeff = distance(init, goal)` heavily penalizes collisions over flight time

## Development Workflow

### Running the Algorithm

```bash
python main.py
```

This launches a tkinter GUI allowing:
- **Random map**: Specify count + sorting method (Crescent/Distance/Random)
- **Load map**: Select `.txt` file with obstacle definitions

Output shows real-time iteration performance and plots best path with vehicle parameters overlay.

### Testing Individual Components

- **Dubins path generation**: See `dubins_path.py` functions `draw_tangent_line()` and `draw_arc_circle()`
- **Collision detection**: `obstacles.py` functions `segment_intersects_circle()` and `circles_intersect()`
- **Vehicle dynamics**: `genetic_base.py` method `MinTurnRadius_DubinsAirplane()` uses physics (bank angle, gravity)

## Project-Specific Patterns

### Encoding Strategy (Critical for Mutations/Crossover)

Each `Individual` stores **three parallel lists** (one per obstacle):
- `tangency`: [RT|LT|NT] approach mode
- `v`: flight speed (derived from physics model)
- `wingspan`: morphing wing setting (0.448–0.667 m)

**Mutation** modifies random obstacle indices:
- 50% chance: randomize tangency for that obstacle
- 50% chance: randomize vehicle parameters (re-runs physics model)

**Crossover** splits both genes at random obstacle index and recombines.

### Obstacle Representation

Obstacles stored as dictionaries: `{'x': int, 'y': int, 'radius': float}`

- Start/goal obstacles always have `radius = 0` (handled specially)
- Turn radii computed relative to these, accounting for wingspan buffer

### Path Representation

Paths stored as `[x_coords, y_coords]` lists built segment-by-segment:
- Each segment = arc (on starting circle) + tangent line + arc (on ending circle)
- Stored in `self.arc_list` and `self.tan_list` for collision checking

## Important Parameters & Global Variables

| File | Parameter | Default | Purpose |
|------|-----------|---------|---------|
| `genetic_algo.py` | `POPULATION_SIZE` | 100 | Must be even |
| `genetic_algo.py` | `GENERATION_COUNT` | 0 | Set via GUI (must override) |
| `genetic_algo.py` | `SELECTION` | "Roulette" | Options: "Roulette", "Tournament", "BestHalf" |
| `genetic_algo.py` | `CROSSOVER_RATE` | 0.7 | Probability of crossover vs. cloning |
| `genetic_algo.py` | `MUTATION_RATE` | 0.3 | Per-iteration mutation probability |
| `obstacles.py` | `B_MIN`, `B_MAX` | 0.448, 0.667 | Morphing wing span range (meters) |
| `genetic_base.py` | `VARIABLE_SPEED` | False | Unused flag for speed optimization |

## Common Pitfalls & Edge Cases

1. **No-turn (NT) obstacles**: When creating paths, algorithm skips consecutive NT obstacles and finds next turn obstacle. If all remaining are NT, behavior undefined (check `read_tangencies()`)
2. **Radius intersections**: `radius_intersect()` checks if expanded obstacle radii overlap; if true, forces next obstacle's tangency to match current (prevents impossible geometry)
3. **Start/goal tangency**: Always forced to RT/LT (no NT), since start has no incoming direction and goal has no outgoing direction
4. **Fitness not tracking best**: In `run_algo()`, best individual is only updated if score improves but best individual itself is recreated each iteration—store best via `best_indv = ranking[0]`
5. **Collision checking asymmetry**: Arc collisions penalize +3, tangent line collisions penalize +6; looping arcs (+1) and tangent buffer breaches (+1) also penalized separately

## File Responsibility Map

| File | Role |
|------|------|
| `main.py` | GUI entry point; orchestrates map load/generation and algorithm launch |
| `genetic_algo.py` | `Population` class; selection, crossover, mutation logic; `run_algo()` main loop |
| `genetic_base.py` | `Individual` class; path construction, collision detection, fitness evaluation |
| `dubins_path.py` | Dubins curve math; generates smooth paths between obstacle tangent points |
| `obstacles.py` | `Circles` class; obstacle generation, collision primitives (circle-circle, line-circle) |
| `plot.py` | Visualization (not analyzed here; assumes matplotlib) |
| `draw_obstacles.py` | Drawing utilities (not analyzed here) |

## Extending This Codebase

- **New selection methods**: Add to `Population.selection()` conditional; implement `self.ind_selected` assignment
- **New mutation strategies**: Modify `Population.mutate()` probabilities or tangency/parameter sampling
- **Physic model improvements**: Update `randomize_vehiclePars()` or `MinTurnRadius_DubinsAirplane()` equations
- **Different path primitives**: Replace dubins with RRT*/PRM but ensure `find_segment()` still populates `arc_list` and `tan_list` for collision checks
