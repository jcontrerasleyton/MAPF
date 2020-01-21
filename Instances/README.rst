## Execute the BASH script "./main.sh" and select one of this opttions:

### 1) Instance_fixer
       * Reorders files (original instances) from Input_original to Input_fix.
       * Files from Input_original have inverted x and y values.
       * x and y values are modified due to the frames added to the maps (+1).

### 2) Instance_generator
       * Generate random instances using maps from Maps_fix.
       * The instances are stored in Input_old.
       * There is the possibility that the goal node cannot be reached from the start node.

### 3) Instance_generator_patched
       * Generate random instances using Maps_fix maps.
       * The instances are stored in Input_patched.
       * The goal node is always reached from the start node.
       * All isolated spaces (without exit) are patched.
       * AUX contains maps patched by Instance_generator_patched.

### 4) Instance_generator_astar
       * Generate random instances using Maps_fix maps.
       * The instances are stored in Input.
       * The goal node is always reached from the start node.
       * It is checked if it is possible to get from star to goal, otherwise, they are generated again.
       * The maps remain intact.
