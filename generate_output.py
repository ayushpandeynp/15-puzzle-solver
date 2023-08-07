import os
for i in range(1, 9):
    print(f"Generating Output{i}.txt")
    os.system(f"python3 solver.py \"Input{i}.txt\" \"Output{i}.txt\"")