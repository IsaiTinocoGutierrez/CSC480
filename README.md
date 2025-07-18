# CSC480
CSC 480 Project 1
**To Run:**
- Run in terminal:
- python make_vacuum_world.py <rows> <columns> <blocked_fraction> <num_dirty> > sample.txt
- Example: python make_vacuum_world.py 5 7 0.2 3 > sample-5x7.txt
- This generates a 5-row by 7-column world with approximately 15% blocked cells and 3
dirty cells, and saves it to sample-5x7.txt.
- **To Run planner.py**
- Run in terminal: python planner.py [algorithm] [world_file]
- Examples:
- python planner.py depth-first sample-5x7.txt
- python planner.py uniform-cost sample-5x7.txt
