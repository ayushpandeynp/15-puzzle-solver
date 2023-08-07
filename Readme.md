# 15-Puzzle Solver Readme
There are two main files: `solver.py` and `generate_output.py`.

## Prerequisites

- Python 3.8.10

## Usage

### solver.py
This script solves a single puzzle and generates an output file.

**<input_file_name>:** The name of the input file containing the puzzle configuration.
**<output_file_name> (optional):** The name of the output file where the solution will be written. Defaults to "output.txt" if not provided.

**Example:**
```python
python3 solver.py "Input1.txt" "Output1.txt"
```

### generate_output.py

This script solves puzzles 1 through 8.

```python
python3 generate_output.py
```

**Example Input File**
The input file should have the following format:

```
1 2 3 4
5 6 7 8
9 10 11 12
13 14 15 0
``````

Use numbers 1 through 15 to represent the tiles of the puzzle.
Use 0 to represent the empty space.

**Output**
For solver.py, the output file will contain the sequence of moves needed to solve the puzzle.
For generate_output.py, the output files will be named as "Output1.txt", "Output2.txt", and so on, corresponding to the input puzzle files.